"""Qt widget for the calibration dashboard.

MoveIt-Setup-Assistant-style layout: a left sidebar picks a step (Start,
Joint Limits, Camera Localization, or one of the accuracy/repeatability/
workspace tests) and the right panel swaps to that step's controls. The
Start tab only offers robot homing; the Joint Limits tab captures the
mechanical stops from a joystick jog session; each test tab is
self-contained (its params, capture
settings, and Start / Continue / Reset / Cancel). A shared strip beneath
the pages holds the status line, log, and post-run result banner.

Workflow (real hardware only):
  1. Terminal 1: bring up the robot with the AprilTag detector
     (`real_bringup.launch.py mode:=tests markers:=true`).
  2. Terminal 2: open this GUI
     (`calibration_gui.launch.py`).
  3. Pick a step in the sidebar, fill in the poses / iteration count, and
     click Start Run. The arm moves to the initial pose, captures a
     baseline reading of the apriltag base->ee transform, then begins the
     iteration loop. At each goal the arm settles and the runner waits for
     a freshly progressed apriltag TF stamp before capturing. Auto-continue
     advances on the next fresh detection; unchecking it falls back to a
     manual Continue click. Cancel aborts the whole run.
  4. Repeatability additionally gates each return-to-home on the detected
     vs URDF Y-Z segment length agreeing within tolerance for the
     configured number of consecutive fresh frames.

UI is built programmatically (no .ui file) for simplicity.
"""

from __future__ import annotations

import html
import re
import shutil
import subprocess
import time
from pathlib import Path
import threading
from typing import Optional

from ament_index_python.packages import get_package_share_directory

from python_qt_binding.QtCore import Signal, Slot, QObject, QTimer, Qt
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QLineEdit,
    QMessageBox, QScrollArea, QSizePolicy, QSplitter,
    QSpinBox, QPushButton, QLabel, QListWidget, QListWidgetItem,
    QStackedWidget, QPlainTextEdit, QProgressBar, QTextEdit,
)


# Status messages matching any of these patterns (case-insensitive)
# get logged in red. Catches IK / motion / settle failures, aborts,
# missing services, stale detections, etc. `error` is matched with word
# boundaries so metric names like `d_error` / `d_urdf` in a normal capture
# line don't get flagged as failures.
_ERROR_PATTERNS = re.compile(
    r'(fail|abort|cannot|invalid|stale|out of reach|\berror\b|empty|no goals'
    r'|not visible|not moving)', re.IGNORECASE)
# Successes (completions, captures, arrivals) get logged in green so
# the operator can scan progress quickly. The match is intentionally
# narrow to avoid colouring routine progress lines.
_SUCCESS_PATTERNS = re.compile(
    r'(\bcompleted\b|\barrived\b|\bcaptured\b|run completed)',
    re.IGNORECASE)

import yaml

from ..runner import (
    CalibrationRunner, CameraCalibrationRunner, RunRequest, TEST_REGISTRY,
    MODE_STAND, MODE_ON_ROBOT,
)
from ..runner.data_writer import load_resume_state, load_sweep_resume_state
# Run discovery only (list_runs + status). Analysis has no rclpy/Qt
# dependencies, so importing it here is safe and keeps the completed-run
# counters consistent with what the notebooks will aggregate.
from ..analysis import loader as _analysis_loader

# Per-test protocol guidance shown on each page. ISO 9283 specifies 30
# cycles per pose; the across-run averaging happens in the notebooks.
_PROTOCOL_NOTES = {
    'noise_gate': (
        'Exp0 step 3 (BLOCKING): ~5 static poses spanning the task '
        'region, ~500 samples each. Validate in noise_gate.ipynb: '
        'effective noise must be <= 1-2 mm per axis before '
        'any sweep runs.'),
    'settle_probe': (
        'Exp0 step 4: settle time is forced to 0 and each visit records '
        'a timestamped burst (~120 samples ~ 4 s at 30 Hz). '
        'settle_probe.ipynb reports the p95 settle time - set it as the '
        'settle time on the other pages.'),
    'workspace_coverage': (
        'Exp0 steps 5-6: generate the grid from the task rectangle '
        '(measure the joint limits first - runbook step 1), then one '
        'run per pass. Pass 2 runs on a different day / after a power '
        'cycle with pass id 2. Interrupted sweeps resume from the '
        'post-run banner.'),
    'repeatability': (
        'Exp0 step 7: one 30-cycle run per anchor point (ISO 9283 - the '
        'same cluster yields accuracy AP and repeatability RP). Use the '
        'anchor picker below; enable the home gate once the tag mounts '
        'are calibrated.'),
    'static_accuracy': (
        'Protocol: 30 cycles per run (ISO 9283), 3 or more independent '
        'runs with re-homing between them. The notebook averages across '
        'runs.'),
    'backlash': (
        'Optional: each cycle approaches every goal from -Y and +Y via '
        'capture-free pre-points; rows are tagged with the approach '
        'direction. Kept available, not scheduled in Exp0.'),
}


DEFAULT_OUTPUT_DIR = '~/workspaces/volcaniarm_ws/experiments/data'

# Sentinel used until the FK service responds with the actual (y, z)
# corresponding to theta=(0, 0). Picked to be obviously a placeholder.
_HOME_FALLBACK = (0.0, 0.5)


class _RunnerBridge(QObject):
    """Marshals runner callbacks (called from worker thread) onto Qt signals."""
    status = Signal(str)
    progress = Signal(int, int)
    finished = Signal(str, str)
    awaiting_continue = Signal(int, int)
    detection_state = Signal(bool, float)
    home_fk_resolved = Signal(float, float)
    # Limit-switch homing completion. (ok, message)
    home_finished = Signal(bool, str)
    # Camera calibration completion. (status, calib_path_or_empty, reason)
    camera_calib_finished = Signal(str, str, str)


class CalibrationDashboardWidget(QWidget):

    # Sidebar rows / stacked-page indices, ordered as the Exp0 protocol
    # runs them (noise gate -> settle probe -> sweep -> anchors). Test
    # pages map to a TEST_REGISTRY key; Start and Camera have no test.
    _PAGE_START = 0
    _PAGE_LIMITS = 1
    _PAGE_CAMERA = 2
    _PAGE_NOISE = 3
    _PAGE_SETTLE = 4
    _PAGE_SWEEP = 5
    _PAGE_REPEAT = 6
    _PAGE_STATIC = 7
    _PAGE_BACKLASH = 8
    _PAGE_TEST_NAME = {
        _PAGE_NOISE: 'noise_gate',
        _PAGE_SETTLE: 'settle_probe',
        _PAGE_SWEEP: 'workspace_coverage',
        _PAGE_REPEAT: 'repeatability',
        _PAGE_STATIC: 'static_accuracy',
        _PAGE_BACKLASH: 'backlash',
    }
    _NAV_LABELS = (
        'Start', 'Joint Limits', 'Camera Localization',
        'Noise Gate', 'Settle Probe', 'Workspace Sweep',
        'Repeatability', 'Static Accuracy', 'Backlash',
    )

    # Warn when the captured poses' binding angles spread more than this
    # before symmetrising — averaging a real asymmetry lets the grid
    # slightly overshoot the tighter poses' true stops.
    _LIMIT_ASYMMETRY_WARN_RAD = 0.05

    # Limits-driven auto-fill: goal-list pages and their recommended
    # pose counts, the marker line that tags auto-written goals text
    # (hand-typed text never starts with it and is never overwritten),
    # and the shipped defaults the no-clobber rules compare against.
    _RECOMMENDED_GOALS_N = {'noise_gate': 5, 'settle_probe': 4,
                            'backlash': 3}
    _RECO_MARKER = '# auto-recommended from joint limits'
    _RECT_DEFAULTS = (-0.40, 0.40, 0.55, 0.85)
    _SETTLE_DEFAULT_S = 2.0
    _HOME_TOL_DEFAULT_MM = 80.0
    _METRICS_DIR = Path(
        '~/workspaces/volcaniarm_ws/experiments/figures/metrics'
    ).expanduser()

    def __init__(self, node):
        super().__init__()
        self.setObjectName('CalibrationDashboardWidget')
        self.setWindowTitle('Volcaniarm Calibration')

        self._node = node

        # Operator override for the EE marker world-orientation prior
        # used by the EE-sweep calibration solver. Format: 'r,p,y' in
        # radians. Empty string -> URDF lookup at run start.
        marker_rpy: Optional[tuple] = None
        try:
            if not node.has_parameter('marker_world_rpy'):
                node.declare_parameter('marker_world_rpy', '')
            raw = node.get_parameter('marker_world_rpy').value
            if isinstance(raw, str) and raw.strip():
                parts = [float(p) for p in raw.split(',')]
                if len(parts) == 3:
                    marker_rpy = (parts[0], parts[1], parts[2])
                else:
                    node.get_logger().warn(
                        f'marker_world_rpy must be "r,p,y" (3 values); '
                        f'got {raw!r}. Falling back to URDF lookup.')
        except Exception as exc:
            node.get_logger().warn(
                f'marker_world_rpy parse failed ({exc}); '
                f'falling back to URDF lookup.')
        self._marker_world_rpy = marker_rpy
        self._bridge = _RunnerBridge()
        self._runner = CalibrationRunner(node)
        self._runner.status_cb = self._bridge.status.emit
        self._runner.progress_cb = self._bridge.progress.emit
        self._runner.finished_cb = lambda p, s: self._bridge.finished.emit(str(p), s)
        self._runner.awaiting_continue_cb = self._bridge.awaiting_continue.emit
        self._runner.detection_state_cb = self._bridge.detection_state.emit
        self._runner.home_finished_cb = (
            lambda ok, msg: self._bridge.home_finished.emit(bool(ok), msg))

        # Cached home (theta=0,0) FK in workspace (y, z); populated
        # asynchronously from the FK service. Used both to seed default
        # spinbox values and to back the per-axis "reset to home" buttons.
        self._home_fk_y: float = _HOME_FALLBACK[0]
        self._home_fk_z: float = _HOME_FALLBACK[1]

        # Auto-continue state: True between an awaiting_continue signal
        # and the operator (or auto-advance) clicking Continue / Cancel.
        # _auto_continue_fresh_since is the monotonic time the current
        # detection started being fresh, or None if not currently fresh.
        # Reset on every new gate so each goal is timed independently.
        self._awaiting_continue: bool = False
        self._auto_continue_fresh_since: Optional[float] = None
        # Path of the most recent run dir, populated on finished. Used
        # by the post-run banner's Delete and Open notebook actions.
        self._last_run_dir: Optional[Path] = None
        self._last_test_name: Optional[str] = None

        # Per-test-page input widgets, keyed by test name. Each entry is a
        # dict of the widgets that page owns (initial_y/z, goal_y/z or
        # goals_edit, iterations, home_*). Shared capture settings
        # (settle/fresh/auto) live once in the run panel instead.
        self._pages_fields: dict = {}

        # Joint Limits page state: latest /joint_states positions (dict
        # name -> rad, replaced atomically by the executor-thread
        # callback and read from a Qt timer), and the list of captured
        # extreme poses. rqt spins the shared node, so the subscription
        # just works.
        self._latest_joints: dict = {}
        self._limit_captures: list = []

        # Last auto-recommended values for the no-clobber rules: an
        # auto-filled widget is only overwritten by a newer
        # recommendation while it still holds the shipped default or the
        # previous auto value - operator edits always survive.
        self._auto_rect: Optional[tuple] = None
        self._auto_goal_center: Optional[tuple] = None
        self._auto_settle_s: Optional[float] = None
        self._auto_home_tol_mm: Optional[float] = None
        self._auto_pass_id: Optional[int] = None
        self._auto_backlash_offset: Optional[float] = None
        from sensor_msgs.msg import JointState
        self._joint_states_sub = node.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        # Camera-localization runner: derives camera-in-base from one
        # AprilTag detection and writes a YAML record. Composes with
        # the test runner -- shares its TF buffer and _stop_event so a
        # single Cancel halts whichever is active.
        self._cam_runner = CameraCalibrationRunner(
            self._runner, marker_world_rpy=self._marker_world_rpy)
        self._cam_runner.status_cb = self._bridge.status.emit
        self._cam_runner.progress_cb = self._bridge.progress.emit
        self._cam_runner.finished_cb = (
            lambda status, path, reason: self._bridge.camera_calib_finished.emit(
                status, str(path) if path else '', reason))

        self._build_ui()
        self._wire_signals()
        self._defaults_from_fk_in_background()

    # -- UI assembly ----------------------------------------------

    def _build_ui(self):
        root = QHBoxLayout(self)

        self._nav = self._build_sidebar()
        root.addWidget(self._nav)

        # Vertical splitter between the page area and the run panel: the
        # operator drags the divider to choose the split; default ~75/25.
        right = QSplitter(Qt.Orientation.Vertical)
        self._pages = QStackedWidget()

        def _scrolled(page: QWidget) -> QScrollArea:
            # Tall pages (sweep grid, repeatability gate) must scroll
            # instead of vertically crushing their rows when the window
            # is short - collapsed spinboxes are unusable.
            sa = QScrollArea()
            sa.setWidgetResizable(True)
            sa.setFrameShape(QFrame.Shape.NoFrame)
            sa.setWidget(page)
            return sa

        self._pages.addWidget(_scrolled(self._build_start_page()))
        self._pages.addWidget(_scrolled(self._build_limits_page()))
        self._pages.addWidget(_scrolled(self._build_camera_page()))
        self._pages.addWidget(_scrolled(self._build_test_page(
            'noise_gate', with_iterations=False,
            with_home_gate=False, goal_mode='list',
            samples_default=500)))
        self._pages.addWidget(_scrolled(self._build_test_page(
            'settle_probe', with_iterations=True,
            with_home_gate=False, goal_mode='list',
            iterations_default=3,
            iterations_label='probes per pose (cycles)',
            samples_default=120, hide_settle=True)))
        self._pages.addWidget(_scrolled(self._build_test_page(
            'workspace_coverage', with_iterations=True,
            with_home_gate=False, goal_mode='list',
            iterations_default=1,
            iterations_label='cycles (full sweeps)',
            with_grid=True, with_pass_meta=True)))
        self._pages.addWidget(_scrolled(self._build_test_page(
            'repeatability', with_iterations=True,
            with_home_gate=True, goal_mode='single',
            iterations_default=30, with_anchors=True)))
        self._pages.addWidget(_scrolled(self._build_test_page(
            'static_accuracy', with_iterations=True,
            with_home_gate=False, goal_mode='single',
            iterations_default=30)))
        self._pages.addWidget(_scrolled(self._build_test_page(
            'backlash', with_iterations=True,
            with_home_gate=False, goal_mode='list',
            iterations_default=5,
            iterations_label='cycles (reps per direction)',
            with_approach_offset=True)))
        self._pages.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        # Floors so neither side can be dragged into an unusable sliver;
        # pages scroll internally past theirs (see _scrolled above).
        self._pages.setMinimumHeight(220)
        right.addWidget(self._pages)
        right.addWidget(self._build_run_panel())
        right.setStretchFactor(0, 3)
        right.setStretchFactor(1, 1)
        right.setChildrenCollapsible(False)
        root.addWidget(right, stretch=1)

        # The run-control widgets (Start/Continue/Reset/Cancel, capture
        # settings, detection, progress) now live per test tab. Bind the
        # self._* names used by the runner-callback slots to a default tab
        # so a stray callback before the first tab switch is harmless;
        # _on_page_changed rebinds them to whichever test tab is active.
        self._bind_run_widgets(self._pages_fields['static_accuracy'])
        self._apply_styles()
        self._apply_measured_joint_limit()

        # Open at a comfortable size instead of the cramped default rqt
        # gives a fresh plugin. The splitter divider above the run panel
        # lets the operator pick the split; only a small log floor is
        # enforced.
        self._log.setMinimumHeight(60)
        self.setMinimumSize(960, 720)
        self.resize(1080, 820)
        # Default divider position: ~75% pages / ~25% run panel.
        right.setSizes([615, 205])

    def _build_sidebar(self) -> QListWidget:
        nav = QListWidget()
        nav.setObjectName('CalibrationNav')
        nav.setFixedWidth(190)
        for label in self._NAV_LABELS:
            QListWidgetItem(label, nav)
        nav.setCurrentRow(self._PAGE_START)
        nav.setStyleSheet(
            'QListWidget { font-size: 15px; }'
            'QListWidget::item { padding: 12px 10px; }'
            'QListWidget::item:selected { background: #4a90d9; color: white; }')
        return nav

    def _build_start_page(self) -> QWidget:
        # Homing is the page's one action, so it sits at the top - the
        # long instructions and the logo scroll below it, never the
        # other way around.
        page = QWidget()
        v = QVBoxLayout(page)
        title = QLabel('Volcaniarm Calibration')
        title.setStyleSheet('font-size: 22px; font-weight: bold;')
        v.addWidget(title)

        instructions = QLabel(
            '<p>Calibrate the real Volcaniarm against AprilTag ground truth.</p>'
            '<p><b>Launch order</b></p>'
            '<ol>'
            '<li>Terminal 1 - robot + camera + AprilTag detector + RViz:<br>'
            '<code>ros2 launch volcaniarm_bringup real_bringup.launch.py '
            'mode:=tests markers:=true</code></li>'
            '<li>Terminal 2 - this GUI:<br>'
            '<code>ros2 launch volcaniarm_calibration calibration_gui.launch.py</code>'
            '</li>'
            '</ol>'
            '<p><b>Steps (left sidebar, in Exp0 protocol order - see '
            'experiments/RUNBOOK.md)</b></p>'
            '<ul>'
            '<li><b>Joint Limits</b> - jog to the mechanical stops with '
            'the joystick and capture the measured joint range '
            '(once).</li>'
            '<li><b>Camera Localization</b> - measure where the camera '
            'sits relative to the arm base before running tests.</li>'
            '<li><b>Noise Gate</b> - static bursts at a few poses; the '
            'blocking measurement-noise validation (report section 0).'
            '</li>'
            '<li><b>Settle Probe</b> - timestamped bursts on arrival; '
            'measures the true settle time (report section 1).</li>'
            '<li><b>Workspace Sweep</b> - serpentine grid over the task '
            'rectangle, one run per pass.</li>'
            '<li><b>Repeatability</b> - 30-cycle cluster per anchor '
            'point (ISO 9283 AP + RP), tag-confirmed home gate.</li>'
            '<li><b>Static Accuracy</b> - one goal, N cycles, returning '
            'to the initial pose each visit.</li>'
            '<li><b>Backlash</b> - approach-direction hysteresis '
            '(optional).</li>'
            '</ul>')
        instructions.setWordWrap(True)
        instructions.setTextFormat(Qt.TextFormat.RichText)

        # Robot homing: the only action on the Start tab. Triggers the
        # limit-switch homing service on volcaniarm_hardware (the arm
        # moves; the seek takes up to ~30 s). Needed when the robot was
        # booted with auto_home:=false. The GUI can't read whether the
        # arm is already homed, so the operator decides; this just offers
        # the button and reports the outcome of the last call.
        home_box = QGroupBox('Robot homing')
        home_outer = QVBoxLayout(home_box)
        self._home_btn = QPushButton('Home robot')
        self._home_btn.setObjectName('primary')
        self._home_btn.setToolTip(
            'Run limit-switch homing (volcaniarm_hardware_interface/home). The arm '
            'seeks its limit switches and re-zeros; takes up to ~30 s. '
            'Use this if the robot booted with auto_home:=false.')
        home_outer.addWidget(self._home_btn)
        self._home_status = QLabel('not homed this session')
        self._home_status.setStyleSheet('color: gray;')
        home_outer.addWidget(self._home_status)
        v.addWidget(home_box)
        v.addWidget(instructions)

        pixmap = self._load_logo_pixmap()
        image = QLabel()
        image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if pixmap is not None:
            image.setPixmap(pixmap)
        else:
            image.setText('(robot image unavailable)')
            image.setStyleSheet('color: gray;')
        v.addWidget(image)
        return page

    # -- Joint Limits page ----------------------------------------

    _ELBOW_JOINTS = ('volcaniarm_right_elbow_joint',
                     'volcaniarm_left_elbow_joint')

    def _build_limits_page(self) -> QWidget:
        # Joystick-driven measurement of the mechanical joint limits
        # (runbook step 1). The operator jogs the arm through ~4 extreme
        # EE poses (roughly the corners of the aimed task rectangle) and
        # captures the joint angles at each; the symmetric limit is the
        # mean over poses of the binding (max |angle|) joint, leaning on
        # the arm's left/right symmetry. Feeds the sweep page and the
        # notebooks.
        page = QWidget()
        v = QVBoxLayout(page)

        instructions = QLabel(
            '<p><b>Measure the mechanical joint limits</b> (once; runbook '
            'step 1).</p>'
            '<ol>'
            '<li>Home the robot (Start page).</li>'
            '<li>Terminal 3 - joystick teleop:<br>'
            '<code>ros2 launch volcaniarm_controllers '
            'joystick_teleop.launch.py</code></li>'
            '<li>Slowly jog the EE to an extreme pose (as far as it '
            'safely goes - first sign of mechanical contact, cable '
            'strain or link-link proximity) and click <b>Capture '
            'pose</b>. Repeat for ~4 poses spanning both sides at the '
            'task heights, roughly the rectangle corners.</li>'
            '<li>Review the derived symmetric limit and <b>Save</b>. '
            'While jogging, also note which stepper sign raises the EE '
            '(firmware direction-comment check).</li>'
            '</ol>')
        instructions.setWordWrap(True)
        instructions.setTextFormat(Qt.TextFormat.RichText)
        v.addWidget(instructions)

        live_box = QGroupBox('Live joint angles')
        live_l = QVBoxLayout(live_box)
        self._limits_live = QLabel('waiting for /joint_states...')
        self._limits_live.setStyleSheet('font-family: monospace;')
        live_l.addWidget(self._limits_live)
        v.addWidget(live_box)

        cap_box = QGroupBox('Captured extreme poses')
        cap_l = QVBoxLayout(cap_box)
        btn_row = QHBoxLayout()
        cap_btn = QPushButton('Capture pose')
        cap_btn.clicked.connect(self._on_capture_pose)
        btn_row.addWidget(cap_btn)
        rm_btn = QPushButton('Remove selected')
        rm_btn.clicked.connect(self._on_remove_capture)
        btn_row.addWidget(rm_btn)
        clear_btn = QPushButton('Clear all')
        clear_btn.clicked.connect(self._on_clear_captures)
        btn_row.addWidget(clear_btn)
        cap_l.addLayout(btn_row)
        self._limits_list = QListWidget()
        self._limits_list.setStyleSheet('font-family: monospace;')
        self._limits_list.setFixedHeight(110)
        cap_l.addWidget(self._limits_list)
        self._limits_summary = QLabel(
            'capture at least 2 poses (4 recommended) to derive the limit')
        self._limits_summary.setWordWrap(True)
        cap_l.addWidget(self._limits_summary)
        v.addWidget(cap_box)

        save_box = QGroupBox('Save')
        save_l = QVBoxLayout(save_box)
        self._limits_save_btn = QPushButton('Save limits')
        self._limits_save_btn.setObjectName('primary')
        self._limits_save_btn.setEnabled(False)
        self._limits_save_btn.setToolTip(
            'Write config/joint_limits.yaml (all captured poses + the '
            'symmetric limit) and prefill the sweep page\'s joint-limit '
            'spinbox. The value saved is the RAW measured stop; the '
            'safety margin stays in the grid generator\'s "limit margin".')
        self._limits_save_btn.clicked.connect(self._on_save_limits)
        save_l.addWidget(self._limits_save_btn)
        self._limits_saved_status = QLabel('')
        self._limits_saved_status.setStyleSheet('color: gray;')
        save_l.addWidget(self._limits_saved_status)
        v.addWidget(save_box)

        # rqt's executor delivers /joint_states off the Qt thread; a
        # plain repaint timer keeps the readout live without cross-thread
        # widget access.
        self._limits_timer = QTimer(self)
        self._limits_timer.timeout.connect(self._update_limits_readout)
        self._limits_timer.start(250)
        return page

    def _on_joint_states(self, msg):
        # Executor thread: atomic dict replace only, no Qt calls.
        self._latest_joints = dict(zip(msg.name, msg.position))

    def _elbow_angles(self) -> Optional[dict]:
        joints = self._latest_joints
        if not all(j in joints for j in self._ELBOW_JOINTS):
            return None
        return {j: joints[j] for j in self._ELBOW_JOINTS}

    @Slot()
    def _update_limits_readout(self):
        angles = self._elbow_angles()
        if angles is None:
            self._limits_live.setText('waiting for /joint_states...')
            return
        self._limits_live.setText('   '.join(
            f'{name.replace("volcaniarm_", "")}: {val:+.4f} rad'
            for name, val in angles.items()))

    @Slot()
    def _on_capture_pose(self):
        angles = self._elbow_angles()
        if angles is None:
            QMessageBox.warning(
                self, 'No joint states',
                'No /joint_states received yet - is the robot bringup '
                'running?')
            return
        self._limit_captures.append(angles)
        self._limits_list.addItem(
            f'pose {len(self._limit_captures)}:  ' + '   '.join(
                f'{name.replace("volcaniarm_", "")}: {val:+.4f}'
                for name, val in angles.items()))
        self._recompute_limit_summary()

    @Slot()
    def _on_remove_capture(self):
        row = self._limits_list.currentRow()
        if row < 0:
            return
        self._limits_list.takeItem(row)
        self._limit_captures.pop(row)
        # Renumber the remaining rows so labels stay pose 1..N.
        for i in range(self._limits_list.count()):
            text = self._limits_list.item(i).text()
            self._limits_list.item(i).setText(
                f'pose {i + 1}:' + text.split(':', 1)[1])
        self._recompute_limit_summary()

    @Slot()
    def _on_clear_captures(self):
        self._limit_captures.clear()
        self._limits_list.clear()
        self._recompute_limit_summary()

    # Angles within this distance of the most extreme captured value are
    # treated as "at the stop" and averaged into that bound; anything
    # further inward is a non-binding interior angle and is ignored.
    _LIMIT_CLUSTER_RAD = 0.1
    # A measured bound smaller than this magnitude gets a warning (real
    # stops can be small - e.g. an inward stop at -0.2 - but a tiny
    # value usually means the arm was not really at the stop).
    _LIMIT_SMALL_BOUND_RAD = 0.3

    def _derive_limits(self):
        """Derive the shared per-motor range [q_min, q_max] from the
        captured poses.

        Mirror symmetry: both motors share the same range (equal
        magnitudes), but within the range min does NOT have to equal
        -max (e.g. [-0.2, 1.06]). At each captured extreme pose one
        motor sits at a stop while the other is interior, so the bounds
        are taken from the CLUSTER of most-extreme angles across all
        captured poses and both motors, averaging within
        _LIMIT_CLUSTER_RAD of the global extreme.

        Returns (q_min, q_max, n_min, n_max, notes) where a bound is
        None when that direction was never captured (no angle with the
        matching sign).
        """
        angles = [v for pose in self._limit_captures
                  for v in pose.values()]
        gmax, gmin = max(angles), min(angles)
        notes = []
        q_max = n_max = None
        if gmax > 0.0:
            cluster = [a for a in angles
                       if a > gmax - self._LIMIT_CLUSTER_RAD]
            q_max, n_max = sum(cluster) / len(cluster), len(cluster)
            if max(cluster) - min(cluster) > self._LIMIT_ASYMMETRY_WARN_RAD:
                notes.append(
                    f'max-bound samples spread '
                    f'{max(cluster) - min(cluster):.3f} rad')
            if q_max < self._LIMIT_SMALL_BOUND_RAD:
                notes.append(
                    f'q_max = {q_max:+.3f} rad is unusually small - '
                    f'confirm the arm was really at its positive stop')
        q_min = n_min = None
        if gmin < 0.0:
            cluster = [a for a in angles
                       if a < gmin + self._LIMIT_CLUSTER_RAD]
            q_min, n_min = sum(cluster) / len(cluster), len(cluster)
            if max(cluster) - min(cluster) > self._LIMIT_ASYMMETRY_WARN_RAD:
                notes.append(
                    f'min-bound samples spread '
                    f'{max(cluster) - min(cluster):.3f} rad')
        return q_min, q_max, n_min, n_max, notes

    def _recompute_limit_summary(self):
        n = len(self._limit_captures)
        if n < 2:
            self._limits_summary.setText(
                'capture at least 2 poses (4 recommended) to derive the '
                'limits')
            self._limits_save_btn.setEnabled(False)
            return
        q_min, q_max, n_min, n_max, notes = self._derive_limits()
        if q_max is None and q_min is None:
            self._limits_summary.setText(
                'no captured angle is near a stop yet - jog further out '
                'before capturing')
            self._limits_save_btn.setEnabled(False)
            return
        parts = []
        if q_max is not None:
            parts.append(f'<b>q_max = {q_max:+.4f} rad</b> '
                         f'({n_max} samples)')
        else:
            parts.append(f'q_max = mirrored ({-q_min:+.4f} rad) - no '
                         f'positive-side extreme captured')
        if q_min is not None:
            parts.append(f'<b>q_min = {q_min:+.4f} rad</b> '
                         f'({n_min} samples)')
        else:
            parts.append(f'q_min = mirrored ({-q_max:+.4f} rad) - no '
                         f'negative-side extreme captured')
        text = ('shared per-motor range (mirror symmetry): '
                + ', '.join(parts))
        for note in notes:
            text += (f'<br><span style="color:#d04b4b;">WARNING: {note} '
                     f'- re-capture the sloppy pose or remove it.</span>')
        self._limits_summary.setText(text)
        self._limits_save_btn.setEnabled(True)

    def _joint_limits_config_path(self) -> Path:
        return Path(
            '~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
            'config/joint_limits.yaml').expanduser()

    @Slot()
    def _on_save_limits(self):
        if len(self._limit_captures) < 2:
            return
        q_min, q_max, _, _, _ = self._derive_limits()
        if q_max is None and q_min is None:
            return
        # An uncaptured direction mirrors the measured one.
        if q_max is None:
            q_max = -q_min
        if q_min is None:
            q_min = -q_max
        path = self._joint_limits_config_path()
        payload = {
            'captured': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'poses': [{k: float(v) for k, v in a.items()}
                      for a in self._limit_captures],
            # RAW measured stops, shared by both motors (mirror
            # symmetry). The grid generator's limit_margin_rad supplies
            # the safety margin - do not pre-subtract it here.
            'q_min_rad': float(q_min),
            'q_max_rad': float(q_max),
            # Half-range equivalent for symmetric consumers (notebook
            # QLIM lobe figures).
            'symmetric_limit_rad': float((q_max - q_min) / 2.0),
        }
        header = ('# Measured mechanical joint limits - written by the '
                  'calibration dashboard\n# Joint Limits page (joystick '
                  'capture flow, runbook step 1). Consumed by the\n# sweep '
                  "page's joint-limit prefill and the notebooks' QLIM.\n")
        path.write_text(header + yaml.safe_dump(payload, sort_keys=False))
        self._limits_saved_status.setText(
            f'saved [{q_min:+.4f}, {q_max:+.4f}] rad to {path.name} '
            f'({payload["captured"]})')
        self._limits_saved_status.setStyleSheet('color: #2e9c4a;')
        self._apply_measured_joint_limit()

    def _apply_measured_joint_limit(self):
        """Prefill the sweep page's joint-limit spinboxes from
        joint_limits.yaml when measured values exist. Called after UI
        build and after settings restore so the measured values always
        win over the placeholders and stale persisted spinboxes. Always
        ends by re-deriving every limits-driven default (goals,
        rectangle, metrics prefills) - even without a yaml, so the
        placeholder-based recommendations still appear."""
        path = self._joint_limits_config_path()
        if not path.exists():
            self._refresh_limit_driven_defaults()
            return
        try:
            data = yaml.safe_load(path.read_text()) or {}
            sym = float(data['symmetric_limit_rad'])
            q_max = float(data.get('q_max_rad', sym))
            q_min = float(data.get('q_min_rad', -sym))
        except Exception as exc:
            self._node.get_logger().warn(
                f'ignoring {path.name}: {exc}')
            self._refresh_limit_driven_defaults()
            return
        sweep = self._pages_fields['workspace_coverage']
        tip = (f'Measured {data.get("captured", "?")} on the Joint Limits '
               f'page (joint_limits.yaml). The grid generator applies the '
               f'"limit margin" below inside this raw stop value.')
        sweep['joint_limit_rad'].setValue(q_max)
        sweep['joint_limit_rad'].setToolTip(tip)
        sweep['joint_limit_min_rad'].setValue(q_min)
        sweep['joint_limit_min_rad'].setToolTip(tip)
        if hasattr(self, '_limits_saved_status'):
            self._limits_saved_status.setText(
                f'saved limits on file: [{q_min:+.4f}, {q_max:+.4f}] rad '
                f'({data.get("captured", "?")})')
            self._limits_saved_status.setStyleSheet('color: #2e9c4a;')
        self._refresh_limit_driven_defaults()

    # -- limits-driven auto-fill ----------------------------------
    #
    # Everything the system has already measured flows forward as an
    # EDITABLE default: goal lists, the sweep rectangle, goal centers,
    # settle time, home tolerance, pass id. The no-clobber rule
    # everywhere: only overwrite a widget still holding the shipped
    # default or the previous auto value; hand-entered values survive.
    # Start Run executes exactly what is on the page.

    def _load_joint_limit_range(self):
        """Measured [q_min, q_max] from joint_limits.yaml, falling back
        to the symmetric value, then the +-1.13 placeholder."""
        try:
            data = yaml.safe_load(
                self._joint_limits_config_path().read_text()) or {}
            sym = float(data['symmetric_limit_rad'])
            return (float(data.get('q_min_rad', -sym)),
                    float(data.get('q_max_rad', sym)))
        except Exception:
            return (-1.13, 1.13)

    def _recommended_goals(self, n: int) -> list:
        """Representative reachable poses for a goals list: the kept
        grid point nearest each spread target over the recommended task
        rectangle (where the tests will actually run), falling back to
        the reachable cloud's bounding box."""
        from ..grid import reachable_cloud, recommended_rectangle
        q_min, q_max = self._load_joint_limit_range()
        kept = reachable_cloud(q_max, joint_limit_min_rad=q_min)
        if not kept:
            return []
        rect = recommended_rectangle(q_max, joint_limit_min_rad=q_min)
        if rect is not None:
            y0, y1, z0, z1 = rect
        else:
            ys = [p[0] for p in kept]
            zs = [p[1] for p in kept]
            y0, y1, z0, z1 = min(ys), max(ys), min(zs), max(zs)
        fracs = {
            # noise gate: center + 4 extremes of the region
            5: [(0.5, 0.5), (0.06, 0.5), (0.94, 0.5),
                (0.5, 0.06), (0.5, 0.94)],
            # settle probe: center, both sides, deep
            4: [(0.5, 0.5), (0.08, 0.4), (0.92, 0.4), (0.5, 0.92)],
            # backlash: lateral spread at mid height
            3: [(0.1, 0.5), (0.5, 0.5), (0.9, 0.5)],
        }[n]
        goals = []
        for fy, fz in fracs:
            ty, tz = y0 + fy * (y1 - y0), z0 + fz * (z1 - z0)
            best = min(kept,
                       key=lambda p: (p[0] - ty) ** 2 + (p[1] - tz) ** 2)
            g = (round(best[0], 3), round(best[1], 3))
            if g not in goals:
                goals.append(g)
        return goals

    def _refresh_recommended_goals(self, force_test: str = None):
        """Fill goal-list editors with limits-derived poses. Overwrites
        only empty or marker-tagged text (force_test overwrites that
        page unconditionally - the per-page Recommend button)."""
        q_min, q_max = self._load_joint_limit_range()
        for test_name, n in self._RECOMMENDED_GOALS_N.items():
            fields = self._pages_fields.get(test_name)
            if not fields or 'goals_edit' not in fields:
                continue
            edit = fields['goals_edit']
            current = edit.toPlainText().strip()
            if (current and not current.startswith(self._RECO_MARKER)
                    and force_test != test_name):
                continue
            try:
                goals = self._recommended_goals(n)
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warn(
                    f'goal recommendation failed: {exc}')
                return
            if not goals:
                continue
            edit.setPlainText(
                f'{self._RECO_MARKER} [{q_min:+.3f}, {q_max:+.3f}] rad '
                f'- edit freely; Start Run executes this list\n'
                + ''.join(f'{y:.3f}, {z:.3f}\n' for y, z in goals))

    @staticmethod
    def _rect_close(a: tuple, b: tuple) -> bool:
        return all(abs(x - y) < 5e-4 for x, y in zip(a, b))

    def _refresh_recommended_rectangle(self, force: bool = False):
        """Prefill the sweep rectangle with the largest rectangle inside
        the measured reachable region, then reseed goal centers and the
        anchor combo. Operator-tuned rectangles survive unless forced."""
        sweep = self._pages_fields.get('workspace_coverage')
        if not sweep or 'grid_y0' not in sweep:
            return
        current = (sweep['grid_y0'].value(), sweep['grid_y1'].value(),
                   sweep['grid_z0'].value(), sweep['grid_z1'].value())
        untouched = (self._rect_close(current, self._RECT_DEFAULTS)
                     or (self._auto_rect is not None
                         and self._rect_close(current, self._auto_rect)))
        if force or untouched:
            try:
                from ..grid import recommended_rectangle
                q_min, q_max = self._load_joint_limit_range()
                rect = recommended_rectangle(
                    q_max, joint_limit_min_rad=q_min)
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warn(
                    f'rectangle recommendation failed: {exc}')
                rect = None
            if rect is not None:
                for key, val in zip(
                        ('grid_y0', 'grid_y1', 'grid_z0', 'grid_z1'),
                        rect):
                    sweep[key].setValue(val)
                    sweep[key].setToolTip(
                        'Auto-recommended: largest rectangle inside the '
                        'measured reachable region - edit freely.')
                self._auto_rect = rect
                # A fresh rectangle means fresh anchors.
                if 'anchor_combo' in self._pages_fields.get(
                        'repeatability', {}):
                    self._on_load_anchors('repeatability')
        self._seed_goal_centers()

    def _seed_goal_centers(self):
        """Default the single-goal pages' goal pose to the sweep
        rectangle's center (home-FK / previous-auto sentinel rule)."""
        sweep = self._pages_fields.get('workspace_coverage')
        if not sweep or 'grid_y0' not in sweep:
            return
        cy = round((sweep['grid_y0'].value()
                    + sweep['grid_y1'].value()) / 2, 3)
        cz = round((sweep['grid_z0'].value()
                    + sweep['grid_z1'].value()) / 2, 3)
        prev = self._auto_goal_center
        for tn in ('repeatability', 'static_accuracy'):
            fields = self._pages_fields.get(tn)
            if not fields or 'goal_y' not in fields:
                continue
            for key, val, idx in (('goal_y', cy, 0), ('goal_z', cz, 1)):
                sb = fields[key]
                sentinels = [_HOME_FALLBACK[idx],
                             (self._home_fk_y, self._home_fk_z)[idx]]
                if prev is not None:
                    sentinels.append(prev[idx])
                if any(abs(sb.value() - s) < 1e-6 for s in sentinels):
                    sb.setValue(val)
                    sb.setToolTip(
                        'Auto-recommended: sweep rectangle center - '
                        'edit freely.')
        self._auto_goal_center = (cy, cz)

    def _load_metrics_file(self, name: str) -> dict:
        try:
            return yaml.safe_load(
                (self._METRICS_DIR / f'{name}.yaml').read_text()) or {}
        except Exception:
            return {}

    def _apply_measured_results(self):
        """Prefill defaults from notebook-saved metrics: settle-time p95
        everywhere, home-gate tolerance from the measured mean error."""
        settle = self._load_metrics_file('settle_probe')
        p95 = settle.get('p95_s')
        if p95 is not None:
            for fields in self._pages_fields.values():
                sb = fields.get('settle_time')
                if sb is None or not sb.isEnabled():
                    continue
                cur = sb.value()
                if (abs(cur - self._SETTLE_DEFAULT_S) < 1e-9
                        or (self._auto_settle_s is not None
                            and abs(cur - self._auto_settle_s) < 1e-9)):
                    sb.setValue(float(p95))
                    sb.setToolTip(
                        f'Measured p95 settle time (settle probe '
                        f'{settle.get("run", "?")}) - edit freely.')
            self._auto_settle_s = float(p95)
        sweep_m = self._load_metrics_file('sweep')
        mean_mm = sweep_m.get('mean_mm')
        if mean_mm is not None:
            sb = self._pages_fields.get(
                'repeatability', {}).get('home_tol_mm')
            if sb is not None:
                suggested = max(20.0, round(1.5 * float(mean_mm), 1))
                cur = sb.value()
                if (abs(cur - self._HOME_TOL_DEFAULT_MM) < 1e-9
                        or (self._auto_home_tol_mm is not None
                            and abs(cur - self._auto_home_tol_mm) < 1e-9)):
                    sb.setValue(suggested)
                    sb.setToolTip(
                        f'Suggested 1.5 x measured mean error '
                        f'({mean_mm} mm, sweep metrics) - edit freely.')
                self._auto_home_tol_mm = suggested

    def _refresh_pass_id(self):
        """Propose the next unused sweep pass id from the runs on disk."""
        sb = self._pages_fields.get(
            'workspace_coverage', {}).get('pass_id')
        if sb is None:
            return
        root = (Path(DEFAULT_OUTPUT_DIR).expanduser()
                / 'workspace_coverage')
        seen = []
        for cfg in root.glob('*/*/config.yaml'):
            try:
                seen.append(int((yaml.safe_load(cfg.read_text())
                                 or {}).get('pass_id', 1)))
            except Exception:  # noqa: BLE001
                continue
        nxt = (max(seen) + 1) if seen else 1
        cur = sb.value()
        if cur == 1 or (self._auto_pass_id is not None
                        and cur == self._auto_pass_id):
            sb.setValue(nxt)
            sb.setToolTip(
                f'Next unused pass id ({len(seen)} sweep runs on disk) '
                f'- edit freely. A resumed sweep keeps the interrupted '
                f"run's pass id.")
        self._auto_pass_id = nxt

    def _sync_backlash_offset(self):
        """Backlash approach offset follows the sweep grid spacing while
        the operator has not touched it."""
        sweep = self._pages_fields.get('workspace_coverage', {})
        sb = self._pages_fields.get('backlash', {}).get(
            'approach_offset_m')
        if sb is None or 'grid_spacing' not in sweep:
            return
        spacing = sweep['grid_spacing'].value()
        cur = sb.value()
        if (abs(cur - 0.05) < 1e-9
                or (self._auto_backlash_offset is not None
                    and abs(cur - self._auto_backlash_offset) < 1e-9)):
            sb.setValue(spacing)
            sb.setToolTip(
                'Follows the sweep grid spacing until edited.')
        self._auto_backlash_offset = spacing

    def _prefill_session_note(self):
        edit = self._pages_fields.get(
            'workspace_coverage', {}).get('session_note')
        if edit is None:
            return
        text = edit.text().strip()
        if not text or re.fullmatch(r'session \d{4}-\d{2}-\d{2}', text):
            edit.setText('session ' + time.strftime('%Y-%m-%d'))

    def _refresh_limit_driven_defaults(self):
        """Single entry point: re-derive every auto-filled default.
        Called after UI build, settings restore, and Save limits."""
        self._refresh_recommended_goals()
        self._refresh_recommended_rectangle()
        self._apply_measured_results()
        self._refresh_pass_id()
        self._sync_backlash_offset()
        self._prefill_session_note()
        self._update_grid_candidates('workspace_coverage')

    def _build_camera_page(self) -> QWidget:
        # Camera localization: measure where the camera is relative to
        # the arm base, using one detection of the base AprilTag. No
        # arm motion, no TF publish (the URDF chain remains the source
        # of truth). Operator uses this to compare measured-vs-URDF
        # before / after re-mounting the camera.
        page = QWidget()
        v = QVBoxLayout(page)
        align_box = QGroupBox('Camera localization')
        align_outer = QVBoxLayout(align_box)
        # URDF-detected mode (camera on stand vs camera on robot).
        # Refreshed on a timer along with the alignment status so it
        # reflects the current TF tree, never a stale cache.
        self._mode_label = QLabel('mode: detecting...')
        self._mode_label.setStyleSheet('color: gray;')
        align_outer.addWidget(self._mode_label)
        self._align_status = QLabel('localization: not run yet')
        self._align_status.setStyleSheet('color: gray;')
        align_outer.addWidget(self._align_status)
        self._calibrate_btn = QPushButton('Calibrate camera')
        self._calibrate_btn.setObjectName('primary')
        self._calibrate_btn.setToolTip(
            'Sweeps the arm through the EE poses in calibration_poses.yaml '
            'and solves for the camera pose. Mode is auto-detected from '
            'the URDF: parent of camera_link is world (stand) or '
            'camera_mount_rev_link (on-robot).')
        align_outer.addWidget(self._calibrate_btn)
        # Cancel aborts an in-flight EE-sweep (the run controls no longer
        # live in a shared bar, so the camera tab needs its own Cancel).
        self._camera_cancel_btn = QPushButton('Cancel')
        self._camera_cancel_btn.setObjectName('danger')
        self._camera_cancel_btn.clicked.connect(self._on_cancel_clicked)
        align_outer.addWidget(self._camera_cancel_btn)
        v.addWidget(align_box)
        return page

    def _build_test_page(self, test_name: str, *, with_iterations: bool,
                         with_home_gate: bool, goal_mode: str,
                         iterations_default: int = 3,
                         iterations_label: str = 'iterations',
                         samples_default: Optional[int] = None,
                         hide_settle: bool = False,
                         with_approach_offset: bool = False,
                         with_pass_meta: bool = False,
                         with_grid: bool = False,
                         with_anchors: bool = False) -> QWidget:
        """Build one test page.

        Each page owns its own pose/goal/iterations/home widgets (a Qt
        widget can only live in one layout, so they can't be shared across
        pages). Widget references are stashed in
        ``self._pages_fields[test_name]`` for the run/reset/seed paths;
        every input widget dropped into ``fields`` is persisted
        automatically by save/restore_settings.

        Optional extras (Exp0): ``samples_default`` adds the per-visit
        burst controls; ``hide_settle`` greys the settle spinbox (the
        settle-probe test forces 0); ``with_approach_offset`` adds the
        backlash pre-point offset; ``with_pass_meta`` adds pass id +
        session note; ``with_grid`` adds the task-rectangle grid
        generator feeding the goals list; ``with_anchors`` adds the
        9-anchor-point picker (rectangle taken from the sweep page).
        """
        page = QWidget()
        v = QVBoxLayout(page)
        fields: dict = {}

        # Protocol note + saved-run counter. Plain labels: intentionally
        # NOT persisted (save/restore only touches input widget types).
        note = _PROTOCOL_NOTES.get(test_name)
        if note:
            note_label = QLabel(note)
            note_label.setWordWrap(True)
            note_label.setStyleSheet('color: gray;')
            v.addWidget(note_label)
        runs_label = QLabel('completed runs saved: ?')
        v.addWidget(runs_label)
        fields['runs_label'] = runs_label

        if with_iterations or with_approach_offset:
            cfg_box = QGroupBox('Test configuration')
            cfg_form = QFormLayout(cfg_box)
            if with_iterations:
                iterations = QSpinBox()
                iterations.setRange(1, 100)
                iterations.setValue(iterations_default)
                cfg_form.addRow(iterations_label, iterations)
                fields['iterations'] = iterations
            if with_approach_offset:
                approach_offset = QDoubleSpinBox()
                approach_offset.setRange(0.01, 0.2)
                approach_offset.setSingleStep(0.01)
                approach_offset.setDecimals(3)
                approach_offset.setSuffix(' m')
                approach_offset.setValue(0.05)
                approach_offset.setToolTip(
                    'Capture-free pre-point offset: each goal is approached '
                    'once from y-offset and once from y+offset. Pre-points '
                    'must themselves be reachable.')
                cfg_form.addRow('approach offset', approach_offset)
                fields['approach_offset_m'] = approach_offset
            v.addWidget(cfg_box)

        if with_home_gate:
            # Home-confirm gate: between iterations the runner checks the
            # detected EE marker matches its URDF-predicted home within a
            # tolerance for `hold` fresh frames. Opt-in and OFF by default:
            # the check compares against the URDF, which carries the
            # placeholder AprilTag-mount bias (a ~cm offset), so a tight
            # tolerance can never pass. Enable it only after calibrating
            # the mounts, or with the tolerance set above the known bias.
            home_box = QGroupBox('Home-confirm gate')
            home_outer = QVBoxLayout(home_box)
            verify_home = QCheckBox('verify home with AprilTag between visits')
            verify_home.setChecked(False)
            verify_home.setToolTip(
                'Off by default: the check compares the detected tool to the '
                'URDF prediction, which still carries the placeholder tag-mount '
                'bias, so it fails until the mounts are calibrated (or the '
                'tolerance is set above the bias).')
            home_outer.addWidget(verify_home)
            home_form = QFormLayout()
            home_tol = QDoubleSpinBox()
            home_tol.setRange(1.0, 200.0)
            home_tol.setSingleStep(5.0)
            home_tol.setDecimals(1)
            home_tol.setSuffix(' mm')
            home_tol.setValue(80.0)
            home_form.addRow('Y-Z segment tol', home_tol)
            home_hold = QSpinBox()
            home_hold.setRange(1, 30)
            home_hold.setValue(5)
            home_form.addRow('hold (consecutive fresh frames)', home_hold)
            home_timeout = QDoubleSpinBox()
            home_timeout.setRange(1.0, 60.0)
            home_timeout.setSingleStep(1.0)
            home_timeout.setDecimals(1)
            home_timeout.setSuffix(' s')
            home_timeout.setValue(10.0)
            home_form.addRow('timeout', home_timeout)
            # Grey the params out until the gate is enabled.
            params_holder = QWidget()
            params_holder.setLayout(home_form)
            params_holder.setEnabled(False)
            home_outer.addWidget(params_holder)
            verify_home.toggled.connect(params_holder.setEnabled)
            fields['verify_home'] = verify_home
            fields['home_tol_mm'] = home_tol
            fields['home_hold_frames'] = home_hold
            fields['home_timeout_s'] = home_timeout
            v.addWidget(home_box)

        # Initial pose (defaults to the workspace (y, z) of theta=(0,0)
        # once the FK service responds; until then a sentinel value).
        # Each row has a small button to snap that axis back to home FK.
        initial_box = QGroupBox('Initial pose (workspace, metres)')
        initial_outer = QVBoxLayout(initial_box)
        initial_form = QFormLayout()
        initial_y = self._make_pose_spinbox(_HOME_FALLBACK[0], lo=-0.4, hi=0.4)
        iy_home = self._make_home_btn()
        iy_home.clicked.connect(
            lambda _=False, sb=initial_y: sb.setValue(self._home_fk_y))
        initial_form.addRow('y', self._row_with_home_btn(initial_y, iy_home))
        initial_z = self._make_pose_spinbox(_HOME_FALLBACK[1], lo=0.1, hi=0.9)
        iz_home = self._make_home_btn()
        iz_home.clicked.connect(
            lambda _=False, sb=initial_z: sb.setValue(self._home_fk_z))
        initial_form.addRow('z', self._row_with_home_btn(initial_z, iz_home))
        initial_outer.addLayout(initial_form)
        # Move-to-initial: send the arm to the typed initial pose without
        # starting a run, so the operator can confirm it's reachable + safe.
        move_btn = QPushButton('Move to initial')
        move_btn.clicked.connect(
            lambda _=False, y=initial_y, z=initial_z:
                self._runner.goto(y.value(), z.value()))
        initial_outer.addWidget(move_btn)
        fields['initial_y'] = initial_y
        fields['initial_z'] = initial_z
        v.addWidget(initial_box)

        if with_grid:
            # Task-rectangle grid generator: fills the goals list below
            # with a filtered serpentine (same in-process kinematics the
            # runner uses, see grid.py). The joint limit is deliberately
            # editable: it is UNMEASURED until the mechanical stops are
            # measured (runbook step 1 / protocol item 6b).
            grid_box = QGroupBox('Sweep grid (task rectangle)')
            grid_outer = QVBoxLayout(grid_box)
            grid_form = QFormLayout()

            def _grid_spin(lo, hi, val, step=0.025, decimals=3, suffix=' m'):
                sb = QDoubleSpinBox()
                sb.setRange(lo, hi)
                sb.setSingleStep(step)
                sb.setDecimals(decimals)
                sb.setSuffix(suffix)
                sb.setValue(val)
                return sb

            grid_y0 = _grid_spin(-0.6, 0.6, -0.40)
            grid_y1 = _grid_spin(-0.6, 0.6, 0.40)
            grid_z0 = _grid_spin(0.1, 1.0, 0.55)
            grid_z1 = _grid_spin(0.1, 1.0, 0.85)
            grid_form.addRow('y0 (left)', grid_y0)
            grid_form.addRow('y1 (right)', grid_y1)
            grid_form.addRow('z0 (top)', grid_z0)
            grid_form.addRow('z1 (bottom)', grid_z1)
            grid_spacing = _grid_spin(0.005, 0.1, 0.025, step=0.005)
            grid_form.addRow('spacing', grid_spacing)
            joint_limit = _grid_spin(0.2, 3.14, 1.13, step=0.01,
                                     suffix=' rad')
            joint_limit.setToolTip(
                'Positive-direction stop. UNMEASURED placeholder: capture '
                'the mechanical stops on the Joint Limits page first '
                '(runbook step 1); the homing switches sit at 1.064 / '
                '1.104 rad, the only measured values so far.')
            grid_form.addRow('joint limit max', joint_limit)
            joint_limit_min = _grid_spin(-3.14, -0.2, -1.13, step=0.01,
                                         suffix=' rad')
            joint_limit_min.setToolTip(
                'Negative-direction stop. Defaults to the mirrored max; '
                'the Joint Limits page fills the measured value when '
                'poses on both sides were captured.')
            grid_form.addRow('joint limit min', joint_limit_min)
            limit_margin = _grid_spin(0.0, 0.3, 0.05, step=0.01,
                                      suffix=' rad')
            grid_form.addRow('limit margin', limit_margin)
            closure_margin = _grid_spin(0.0, 0.1, 0.02, step=0.005)
            closure_margin.setToolTip(
                'Minimum distance from the stretched (type-2) singularity '
                'where the distal links go collinear.')
            grid_form.addRow('closure margin', closure_margin)
            grid_outer.addLayout(grid_form)
            gen_btn = QPushButton('Generate grid into goals list')
            gen_btn.clicked.connect(
                lambda _=False, tn=test_name: self._on_generate_grid(tn))
            grid_outer.addWidget(gen_btn)
            rect_btn = QPushButton('Recommend rectangle from joint limits')
            rect_btn.setToolTip(
                'Set the rectangle to the largest one that fits inside '
                'the measured reachable region (overwrites the values '
                'above).')
            rect_btn.clicked.connect(
                lambda _=False:
                self._refresh_recommended_rectangle(force=True))
            grid_outer.addWidget(rect_btn)
            # Live size estimate: candidate count updates on any
            # rectangle/spacing edit; kept count + minutes appear after
            # Generate runs the filter.
            grid_estimate = QLabel('')
            grid_estimate.setStyleSheet('color: gray;')
            grid_outer.addWidget(grid_estimate)
            fields['grid_estimate'] = grid_estimate
            for sb in (grid_y0, grid_y1, grid_z0, grid_z1, grid_spacing):
                sb.valueChanged.connect(
                    lambda _=0.0, tn=test_name:
                    self._update_grid_candidates(tn))
            grid_spacing.valueChanged.connect(
                lambda _=0.0: self._sync_backlash_offset())
            fields['grid_y0'] = grid_y0
            fields['grid_y1'] = grid_y1
            fields['grid_z0'] = grid_z0
            fields['grid_z1'] = grid_z1
            fields['grid_spacing'] = grid_spacing
            fields['joint_limit_rad'] = joint_limit
            fields['joint_limit_min_rad'] = joint_limit_min
            fields['limit_margin_rad'] = limit_margin
            fields['closure_margin_m'] = closure_margin
            v.addWidget(grid_box)

        if goal_mode == 'single':
            goal_box = QGroupBox('Goal pose (workspace, metres)')
            goal_form = QFormLayout(goal_box)
            goal_y = self._make_pose_spinbox(_HOME_FALLBACK[0], lo=-0.4, hi=0.4)
            goal_form.addRow('y', goal_y)
            goal_z = self._make_pose_spinbox(_HOME_FALLBACK[1], lo=0.1, hi=0.9)
            goal_form.addRow('z', goal_z)
            fields['goal_y'] = goal_y
            fields['goal_z'] = goal_z
            if with_anchors:
                # 9-anchor picker: corners + edge mids + center of the
                # task rectangle (taken live from the sweep page's grid
                # fields), each reachability-checked independently.
                # Selecting an anchor fills the goal spinboxes; one
                # 30-cycle run per anchor.
                anchor_row = QHBoxLayout()
                anchor_combo = QComboBox()
                anchor_combo.setPlaceholderText('anchor points...')
                load_btn = QPushButton('Load anchors')
                load_btn.setToolTip(
                    'Compute the 9 anchor points from the task rectangle '
                    'on the Workspace Sweep page (corners + edge mids + '
                    'center, inset one grid spacing).')
                load_btn.clicked.connect(
                    lambda _=False, tn=test_name: self._on_load_anchors(tn))
                anchor_combo.activated.connect(
                    lambda _idx, tn=test_name: self._on_anchor_selected(tn))
                anchor_row.addWidget(anchor_combo, stretch=1)
                anchor_row.addWidget(load_btn)
                goal_form.addRow('anchors', anchor_row)
                fields['anchor_combo'] = anchor_combo
            v.addWidget(goal_box)
        else:
            goals_box = QGroupBox(
                'Goals list (workspace, metres) - one "y, z" per line')
            goals_outer = QVBoxLayout(goals_box)
            goals_edit = QPlainTextEdit()
            goals_edit.setPlaceholderText(
                '0.0, 0.5\n0.1, 0.5\n-0.1, 0.5\n0.0, 0.6')
            # High cap: a full 25 mm sweep grid is 400+ lines (plus the
            # provenance comment header the generator writes).
            goals_edit.setMaximumBlockCount(2000)
            goals_outer.addWidget(goals_edit)
            fields['goals_edit'] = goals_edit
            if test_name in self._RECOMMENDED_GOALS_N:
                reco_btn = QPushButton(
                    'Recommend goals from joint limits')
                reco_btn.setToolTip(
                    'Replace the list with poses derived from the '
                    'measured joint range (overwrites edits on this '
                    'page). Start Run always executes the list as '
                    'shown.')
                reco_btn.clicked.connect(
                    lambda _=False, tn=test_name:
                    self._refresh_recommended_goals(force_test=tn))
                goals_outer.addWidget(reco_btn)
            v.addWidget(goals_box)

        # Reachability guard: every pose edit re-checks IK (in-process,
        # cheap) after a short debounce, and Start refuses to launch a
        # run with an unreachable pose. Catches typos before the arm
        # moves instead of aborting mid-run.
        reach_label = QLabel('reachability: checking...')
        reach_label.setWordWrap(True)
        v.addWidget(reach_label)
        fields['reach_label'] = reach_label
        reach_timer = QTimer(self)
        reach_timer.setSingleShot(True)
        reach_timer.setInterval(250)
        reach_timer.timeout.connect(
            lambda tn=test_name: self._refresh_reachability(tn))
        fields['reach_timer'] = reach_timer
        pose_boxes = [initial_y, initial_z]
        if goal_mode == 'single':
            pose_boxes += [fields['goal_y'], fields['goal_z']]
        for sb in pose_boxes:
            sb.valueChanged.connect(
                lambda _=0.0, t=reach_timer: t.start())
        if goal_mode != 'single':
            fields['goals_edit'].textChanged.connect(
                lambda t=reach_timer: t.start())
        reach_timer.start()

        # Capture settings + run controls live on the tab itself so each
        # test is self-contained. The widgets are per-tab instances (a Qt
        # widget can only sit in one layout); the runner-callback slots
        # reach the active tab's set via _bind_run_widgets on tab switch.
        cap_box = QGroupBox('Capture settings')
        cap_form = QFormLayout(cap_box)
        settle_time = QDoubleSpinBox()
        settle_time.setRange(0.0, 10.0)
        settle_time.setSingleStep(0.5)
        settle_time.setDecimals(1)
        settle_time.setValue(0.0 if hide_settle else 2.0)
        if hide_settle:
            # The settle-probe test forces settle_time = 0 (capturing
            # starts on arrival); show the value greyed so the operator
            # sees why there is no settle here.
            settle_time.setEnabled(False)
            settle_time.setToolTip(
                'Forced to 0 by this test: the burst starting on arrival '
                'IS the settle measurement.')
        cap_form.addRow('settle time (s)', settle_time)
        if samples_default is not None:
            samples = QSpinBox()
            samples.setRange(1, 2000)
            samples.setValue(samples_default)
            samples.setToolTip(
                'Detections captured per visit, each individually gated '
                'on a fresh TF stamp (N samples = N distinct detections).')
            cap_form.addRow('samples per visit', samples)
            sample_period = QDoubleSpinBox()
            sample_period.setRange(0.0, 1.0)
            sample_period.setSingleStep(0.05)
            sample_period.setDecimals(2)
            sample_period.setValue(0.0)
            sample_period.setToolTip(
                'Minimum spacing between burst samples; 0 = as fast as '
                'fresh detections arrive.')
            cap_form.addRow('min sample period (s)', sample_period)
            fields['samples_per_capture'] = samples
            fields['sample_min_period_s'] = sample_period
        fresh_window = QDoubleSpinBox()
        fresh_window.setRange(0.1, 2.0)
        fresh_window.setSingleStep(0.1)
        fresh_window.setDecimals(2)
        fresh_window.setValue(0.5)
        cap_form.addRow('detection fresh window (s)', fresh_window)
        # How long the capture waits after settle for a detection newer
        # than the pre-settle one. Capped so a very long timeout can't
        # mask a marginal detection setup; the default rides out the
        # multi-second detector gaps observed on hardware.
        det_timeout = QDoubleSpinBox()
        det_timeout.setRange(0.5, 15.0)
        det_timeout.setSingleStep(0.5)
        det_timeout.setDecimals(1)
        det_timeout.setValue(5.0)
        det_timeout.setToolTip(
            'Abort budget for a fresh detection after the arm settles. '
            'Raise for a gappy detector; if double digits are needed, '
            'fix lighting / exposure / tag angle instead. A run that '
            'still aborts can be resumed from the post-run banner.')
        cap_form.addRow('detection timeout (s)', det_timeout)
        # Auto-continue: when checked, the runner auto-advances at each
        # Continue gate once detection has been continuously fresh for
        # `fresh-hold` seconds. Cancel still aborts immediately.
        auto_continue = QCheckBox('auto-continue when detection fresh')
        auto_continue.setChecked(True)
        cap_form.addRow(auto_continue)
        auto_hold = QDoubleSpinBox()
        auto_hold.setRange(0.2, 5.0)
        auto_hold.setSingleStep(0.1)
        auto_hold.setDecimals(2)
        auto_hold.setValue(1.0)
        cap_form.addRow('auto-continue fresh-hold (s)', auto_hold)
        fields['settle_time'] = settle_time
        fields['fresh_window'] = fresh_window
        fields['det_timeout'] = det_timeout
        fields['auto_continue'] = auto_continue
        fields['auto_hold'] = auto_hold
        v.addWidget(cap_box)

        if with_pass_meta:
            # Sweep pass metadata, recorded in the run's config.yaml.
            # Pass 2 of the Exp0 serpentine runs on a different day /
            # after a power cycle; the analysis pools passes by pass id.
            pass_box = QGroupBox('Sweep pass')
            pass_form = QFormLayout(pass_box)
            pass_id = QSpinBox()
            pass_id.setRange(1, 20)
            pass_id.setValue(1)
            pass_id.setToolTip(
                'Bump for each independent pass over the same grid '
                '(different day / power cycle). A resumed sweep keeps '
                'the pass id of the interrupted run.')
            pass_form.addRow('pass id', pass_id)
            session_note = QLineEdit()
            session_note.setPlaceholderText(
                'free-form session note (lighting, temperature, ...)')
            pass_form.addRow('session note', session_note)
            fields['pass_id'] = pass_id
            fields['session_note'] = session_note
            v.addWidget(pass_box)

        btn_row = QHBoxLayout()
        start_btn = QPushButton('Start Run')
        start_btn.setObjectName('primary')
        start_btn.clicked.connect(self._on_start_clicked)
        continue_btn = QPushButton('Continue')
        continue_btn.setEnabled(False)
        continue_btn.clicked.connect(self._on_continue_clicked)
        # Reset = abort current run (stop in place) then drive arm back
        # to the typed initial pose. Distinct from Cancel which only stops.
        reset_btn = QPushButton('Reset')
        reset_btn.clicked.connect(self._on_reset_clicked)
        # Cancel = emergency stop. Arm halts wherever it is.
        cancel_btn = QPushButton('Cancel')
        cancel_btn.setObjectName('danger')
        cancel_btn.clicked.connect(self._on_cancel_clicked)
        for b in (start_btn, continue_btn, reset_btn, cancel_btn):
            btn_row.addWidget(b)
        v.addLayout(btn_row)
        fields['start_btn'] = start_btn
        fields['continue_btn'] = continue_btn
        fields['reset_btn'] = reset_btn
        fields['cancel_btn'] = cancel_btn

        detection_label = QLabel('detection: idle')
        v.addWidget(detection_label)
        fields['detection_label'] = detection_label
        progress = QProgressBar()
        progress.setRange(0, 1)
        progress.setValue(0)
        v.addWidget(progress)
        fields['progress'] = progress

        self._pages_fields[test_name] = fields
        return page

    def _build_run_panel(self) -> QWidget:
        """Shared feedback strip beneath the stacked pages: status line,
        log, and the post-run banner. The run controls + capture settings
        + progress/detection live per test tab instead."""
        panel = QWidget()
        v = QVBoxLayout(panel)

        self._status_label = QLabel('idle')
        self._status_label.setStyleSheet('font-weight: bold;')
        v.addWidget(self._status_label)

        # QTextEdit (rich text) instead of QPlainTextEdit so each line
        # can be coloured by severity. Maximum block count keeps the
        # widget bounded; the ring buffer behaviour is identical.
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.document().setMaximumBlockCount(1000)

        # Log header row: a "Log" label and a compact Clear button that
        # empties the log without affecting a run. Built after self._log
        # exists so the Clear button can bind to it; added above the log.
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel('Log'))
        log_header.addStretch(1)
        clear_btn = QPushButton('Clear')
        clear_btn.setObjectName('compact')
        clear_btn.setToolTip('Clear the log messages below')
        clear_btn.clicked.connect(self._log.clear)
        log_header.addWidget(clear_btn)
        v.addLayout(log_header)
        v.addWidget(self._log, stretch=1)

        # Post-run banner: shown after every finalize so the operator
        # can triage the result (Keep / Delete / Open notebook).
        self._banner = self._build_banner()
        v.addWidget(self._banner)
        self._banner.setVisible(False)
        return panel

    def _build_banner(self) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        outer = QVBoxLayout(frame)
        # Status pill: a single label with bold text and a coloured
        # background that swaps green/yellow/red on each finalize.
        self._banner_status = QLabel()
        self._banner_status.setStyleSheet(
            'font-weight: bold; padding: 4px;')
        outer.addWidget(self._banner_status)
        self._banner_path = QLabel()
        self._banner_path.setWordWrap(True)
        self._banner_path.setStyleSheet('color: gray;')
        outer.addWidget(self._banner_path)
        btn_row = QHBoxLayout()
        self._banner_keep = QPushButton('Keep')
        self._banner_resume = QPushButton('Resume run')
        self._banner_delete = QPushButton('Delete run')
        self._banner_open = QPushButton('Open folder')
        self._banner_open.setToolTip(
            'Open the run directory (config.yaml + CSVs). Evaluate runs '
            "in the test's notebook under experiments/notebooks/ "
            '(noise_gate, settle_probe, workspace_sweep, ...).')
        self._banner_keep.clicked.connect(self._on_banner_keep)
        self._banner_resume.clicked.connect(self._on_banner_resume)
        self._banner_delete.clicked.connect(self._on_banner_delete)
        self._banner_open.clicked.connect(self._on_banner_open_folder)
        btn_row.addWidget(self._banner_keep)
        btn_row.addWidget(self._banner_resume)
        btn_row.addWidget(self._banner_delete)
        btn_row.addWidget(self._banner_open)
        outer.addLayout(btn_row)
        return frame

    def _make_pose_spinbox(self, default: float, lo: float, hi: float
                            ) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setSingleStep(0.05)
        sb.setDecimals(3)
        sb.setValue(default)
        return sb

    def _make_home_btn(self) -> QPushButton:
        """Compact button used inline to snap a pose axis to the home FK."""
        btn = QPushButton('home')
        btn.setObjectName('compact')
        btn.setToolTip('Reset this axis to the home FK value (theta=0,0)')
        # Maximum size policy clamps the width to the button's content
        # sizeHint (evaluated after styling, so the label + compact padding
        # always fit) while preventing it from expanding to fill the row.
        btn.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
        return btn

    def _row_with_home_btn(self, spinbox: QDoubleSpinBox,
                           home_btn: QPushButton) -> QHBoxLayout:
        row = QHBoxLayout()
        # Spinbox takes all the slack; the home button stays at its
        # content width so its label isn't clipped and it doesn't sprawl.
        row.addWidget(spinbox, stretch=1)
        row.addWidget(home_btn, stretch=0)
        return row

    def _logo_image_path(self) -> Optional[Path]:
        """Resolve the start-page robot image (share dir, then source tree)."""
        candidates: list = []
        try:
            share = Path(get_package_share_directory('volcaniarm_calibration'))
            candidates.append(share / 'resource' / 'volcaniarm_urdf_img.jpeg')
        except Exception:
            pass
        # Source-tree fallback: the widget lives at
        # <pkg>/volcaniarm_calibration/rqt/this_file.py; parents[2] is the
        # package root, where a resource/ dir holds the image.
        pkg_root = Path(__file__).resolve().parents[2]
        candidates.append(pkg_root / 'resource' / 'volcaniarm_urdf_img.jpeg')
        for path in candidates:
            if path.exists():
                return path
        return None

    def _load_logo_pixmap(self) -> Optional[QPixmap]:
        path = self._logo_image_path()
        if path is None:
            return None
        pixmap = QPixmap(str(path))
        if pixmap.isNull():
            return None
        return pixmap.scaledToWidth(460, Qt.TransformationMode.SmoothTransformation)

    def _apply_styles(self):
        """One cohesive stylesheet for the whole dashboard.

        Theme-neutral: uses palette roles for borders so it reads well in
        both light and dark Qt themes. Primary actions (Home / Start Run /
        Calibrate) and the Cancel (danger) buttons are keyed by objectName.
        """
        self.setStyleSheet('''
            QWidget { font-size: 14px; }
            QGroupBox {
                margin-top: 12px;
                border: 1px solid palette(mid);
                border-radius: 6px;
                padding: 10px 8px 8px 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                font-weight: bold;
            }
            /* Explicit border + background so plain buttons keep visible
               button chrome (a stylesheet with border-radius alone drops
               Qt's native rendering and the button looks like flat text). */
            QPushButton {
                padding: 6px 14px;
                min-height: 24px;
                border: 1px solid palette(mid);
                border-radius: 4px;
                background-color: palette(button);
            }
            QPushButton:hover { background-color: palette(light); }
            QPushButton:pressed { background-color: palette(mid); }
            QPushButton:disabled { color: palette(mid); }
            /* Compact buttons (inline "home" snaps, Clear log): tight
               padding + no min-height so short labels aren't clipped. */
            QPushButton#compact {
                padding: 3px 10px;
                min-height: 0;
            }
            QPushButton#primary {
                background-color: #4a90d9;
                color: white;
                font-weight: bold;
                border: 1px solid #3a7bc0;
            }
            QPushButton#primary:hover { background-color: #3a7bc0; }
            QPushButton#primary:disabled {
                background-color: #9bbfe0; color: #eef; border: none;
            }
            QPushButton#danger {
                background-color: #c0504b;
                color: white;
                border: 1px solid #a5423d;
            }
            QPushButton#danger:hover { background-color: #a5423d; }
        ''')

    def _wire_signals(self):
        # Per-tab run buttons (Start/Continue/Reset/Cancel) are wired in
        # _build_test_page. Here we wire the single-instance controls.
        self._home_btn.clicked.connect(self._on_home_clicked)
        self._calibrate_btn.clicked.connect(self._on_calibrate_clicked)
        # Refresh the alignment status label periodically so it picks
        # up new result.yaml files written between dashboard sessions.
        self._align_timer = QTimer(self)
        self._align_timer.setInterval(2000)
        self._align_timer.timeout.connect(self._refresh_alignment_state)
        self._align_timer.start()
        self._refresh_alignment_state()
        # Sidebar navigation: swap the stacked page and reset run state.
        self._nav.currentRowChanged.connect(self._pages.setCurrentIndex)
        self._nav.currentRowChanged.connect(self._on_page_changed)
        self._bridge.status.connect(self._on_status)
        self._bridge.progress.connect(self._on_progress)
        self._bridge.finished.connect(self._on_finished)
        self._bridge.awaiting_continue.connect(self._on_awaiting_continue)
        self._bridge.detection_state.connect(self._on_detection_state)
        self._bridge.home_fk_resolved.connect(self._on_home_fk_resolved)
        self._bridge.home_finished.connect(self._on_home_finished)
        self._bridge.camera_calib_finished.connect(
            self._on_camera_calib_finished)

    # -- page / test helpers --------------------------------------

    def _current_test_name(self) -> Optional[str]:
        return self._PAGE_TEST_NAME.get(self._nav.currentRow())

    def _active_fields(self) -> Optional[dict]:
        name = self._current_test_name()
        return self._pages_fields.get(name) if name else None

    # Run-control widget keys stored in each test tab's `fields` bundle.
    # These are per-tab instances; _bind_run_widgets points the self._*
    # names (used by the runner-callback slots) at the active tab's set.
    _RUN_WIDGET_KEYS = (
        'settle_time', 'fresh_window', 'auto_continue', 'auto_hold',
        'start_btn', 'continue_btn', 'reset_btn', 'cancel_btn',
        'detection_label', 'progress',
    )

    def _bind_run_widgets(self, fields: dict):
        """Point the self._* run-control names at one test tab's widgets."""
        self._settle_time = fields['settle_time']
        self._fresh_window = fields['fresh_window']
        self._auto_continue = fields['auto_continue']
        self._auto_hold = fields['auto_hold']
        self._start_btn = fields['start_btn']
        self._continue_btn = fields['continue_btn']
        self._reset_btn = fields['reset_btn']
        self._cancel_btn = fields['cancel_btn']
        self._detection_label = fields['detection_label']
        self._progress = fields['progress']

    @Slot(int)
    def _on_page_changed(self, row: int):
        # Landing on a test page implies the operator is starting fresh:
        # rebind the run-control widgets to that tab, cancel anything still
        # running, and reset the run state. Navigating to Start / Camera
        # does NOT cancel, so peeking at another page can't silently abort
        # an active run. cancel() is a no-op when nothing is running.
        name = self._PAGE_TEST_NAME.get(row)
        if name is not None:
            self._bind_run_widgets(self._pages_fields[name])
            self._runner.cancel()
            self._reset_ui_state(status='idle')
            self._refresh_run_counts()
            self._refresh_reachability(name)
            if name == 'workspace_coverage':
                # New runs may have landed since the last visit.
                self._refresh_pass_id()

    # -- reachability guard / run counters -------------------------

    def _reachability_problems(self, test_name: str) -> list:
        """Collect human-readable reachability problems for a page.

        Checks the initial pose and every goal via the runner's
        in-process IK. Returns an empty list when everything is
        reachable; goal-text parse errors are reported as problems too
        so the guard covers the workspace multi-goal editor.
        """
        fields = self._pages_fields[test_name]
        problems: list = []
        iy, iz = fields['initial_y'].value(), fields['initial_z'].value()
        ok, reason = self._runner.check_reachable(iy, iz)
        if not ok:
            problems.append(f'initial pose ({iy:.3f}, {iz:.3f}): {reason}')
        if 'goals_edit' in fields:
            try:
                goals = self._parse_goals_text(
                    fields['goals_edit'].toPlainText())
            except ValueError as exc:
                problems.append(f'goals list: {exc}')
                return problems
            for idx, (gy, gz) in enumerate(goals, start=1):
                ok, reason = self._runner.check_reachable(gy, gz)
                if not ok:
                    problems.append(
                        f'goal {idx} ({gy:.3f}, {gz:.3f}): {reason}')
        else:
            gy, gz = fields['goal_y'].value(), fields['goal_z'].value()
            ok, reason = self._runner.check_reachable(gy, gz)
            if not ok:
                problems.append(f'goal ({gy:.3f}, {gz:.3f}): {reason}')
        return problems

    def _refresh_reachability(self, test_name: str):
        fields = self._pages_fields.get(test_name)
        if fields is None or 'reach_label' not in fields:
            return
        label = fields['reach_label']
        problems = self._reachability_problems(test_name)
        if not problems:
            label.setText('reachability: all poses reachable')
            label.setStyleSheet('color: #2e9c4a;')
        else:
            shown = problems[:3]
            if len(problems) > len(shown):
                shown.append(f'... and {len(problems) - len(shown)} more')
            label.setText('reachability: ' + '; '.join(shown))
            label.setStyleSheet('color: #d04b4b;')

    def _refresh_run_counts(self):
        """Update every test page's completed-run counter from disk."""
        root = Path(DEFAULT_OUTPUT_DIR).expanduser()
        for test_name, fields in self._pages_fields.items():
            label = fields.get('runs_label')
            if label is None:
                continue
            try:
                runs = _analysis_loader.list_runs(test_name, root)
                n = sum(1 for r in runs
                        if _analysis_loader._safe_status(r) == 'completed')  # noqa: SLF001
                label.setText(f'completed runs saved: {n}')
            except Exception as exc:  # noqa: BLE001
                label.setText(f'completed runs saved: ? ({exc})')

    def _log_msg(self, msg: str):
        """Append a status line to the log box, coloured by severity.

        Errors (IK / motion / abort / stale) render red, completions
        / arrivals / captures render green, everything else uses the
        Qt theme default. HTML special characters are escaped so log
        text from arbitrary sources can't accidentally inject markup.
        """
        escaped = html.escape(msg)
        if _ERROR_PATTERNS.search(msg):
            html_line = (
                f'<span style="color: #d04b4b;">{escaped}</span>')
        elif _SUCCESS_PATTERNS.search(msg):
            html_line = (
                f'<span style="color: #2e9c4a;">{escaped}</span>')
        else:
            html_line = escaped
        self._log.append(html_line)

    def _reset_ui_state(self, status: str = 'idle'):
        """Clear progress / detection / button state so the dashboard
        is ready for a fresh run. Called from finished, reset, cancel,
        and page-switch paths."""
        self._start_btn.setEnabled(True)
        self._continue_btn.setEnabled(False)
        self._progress.setValue(0)
        self._detection_label.setText('detection: idle')
        self._detection_label.setStyleSheet('')
        self._status_label.setText(status)

    # -- startup helpers ------------------------------------------

    def _defaults_from_fk_in_background(self):
        """Resolve theta=(0, 0) -> (y, z) via FK and populate defaults.

        The FK client may take a few seconds to come up after the
        dashboard loads (controller spinup, lifecycle ordering, etc.).
        Doing this in a background thread keeps the UI responsive; the
        result is delivered back via a Qt signal so the spinbox writes
        happen on the main thread.
        """
        def worker():
            xyz = self._runner._call_fk(0.0, 0.0)  # noqa: SLF001
            if xyz is not None:
                self._bridge.home_fk_resolved.emit(float(xyz[1]), float(xyz[2]))
        threading.Thread(target=worker, daemon=True).start()

    @Slot(float, float)
    def _on_home_fk_resolved(self, y: float, z: float):
        # Cache the resolved home values so the per-axis "home" buttons
        # always have the right target, even after the user changes the
        # spinboxes.
        self._home_fk_y = y
        self._home_fk_z = z
        # Only overwrite spinboxes still sitting at the fallback sentinel
        # so a late FK reply can't clobber a value the operator (or a
        # restored setting) already typed. Every per-page initial/goal
        # spinbox is seeded, not just one page's.
        for fields in self._pages_fields.values():
            y_boxes = [fields['initial_y']]
            z_boxes = [fields['initial_z']]
            if 'goal_y' in fields:
                y_boxes.append(fields['goal_y'])
            if 'goal_z' in fields:
                z_boxes.append(fields['goal_z'])
            for sb in y_boxes:
                if abs(sb.value() - _HOME_FALLBACK[0]) < 1e-9:
                    sb.setValue(y)
            for sb in z_boxes:
                if abs(sb.value() - _HOME_FALLBACK[1]) < 1e-9:
                    sb.setValue(z)

    # -- button slots --------------------------------------------

    @Slot()
    def _on_start_clicked(self):
        test_name = self._current_test_name()
        if test_name is None:
            self._log_msg('select a test in the sidebar first')
            self._status_label.setText('select a test page to start a run')
            return
        fields = self._pages_fields[test_name]
        cls = TEST_REGISTRY[test_name]
        if 'goals_edit' in fields:
            try:
                goals = self._parse_goals_text(
                    fields['goals_edit'].toPlainText())
            except ValueError as exc:
                self._log_msg(f'goals parse error: {exc}')
                self._status_label.setText(f'cannot start: {exc}')
                return
            if not goals:
                self._log_msg('goals list is empty; nothing to run')
                self._status_label.setText('cannot start: empty goals list')
                return
        else:
            goals = [(fields['goal_y'].value(), fields['goal_z'].value())]
        # Reachability guard: refuse to start rather than letting the
        # runner abort after the arm already started moving.
        problems = self._reachability_problems(test_name)
        if problems:
            for p in problems:
                self._log_msg(f'cannot start: {p}')
            self._status_label.setText('cannot start: unreachable pose(s)')
            self._refresh_reachability(test_name)
            return
        num_cycles = fields['iterations'].value() if 'iterations' in fields else 1
        # The runner executes the test's iter_visits() pattern, so
        # `targets` is authoritative; request.goals is retained only
        # for the config.yaml record.
        extra = {}
        if 'verify_home' in fields:  # repeatability page: opt-in home gate
            extra['verify_home_with_tag'] = fields['verify_home'].isChecked()
        if 'approach_offset_m' in fields:  # backlash page
            extra['approach_offset_m'] = fields['approach_offset_m'].value()
        try:
            test = cls(
                targets=goals,
                num_cycles=num_cycles,
                settle_time=fields['settle_time'].value(),
                return_home_between_targets=True,
                **extra,
            )
        except ValueError as exc:
            self._log_msg(f'cannot start: {exc}')
            self._status_label.setText(f'cannot start: {exc}')
            return
        # Optional per-page params: presence in `fields` decides; the
        # RunRequest dataclass supplies sensible defaults otherwise.
        req_kwargs = {}
        if 'home_tol_mm' in fields:
            req_kwargs.update(
                home_tol_m=fields['home_tol_mm'].value() / 1000.0,
                home_hold_frames=fields['home_hold_frames'].value(),
                home_timeout_s=fields['home_timeout_s'].value(),
            )
        if 'samples_per_capture' in fields:
            req_kwargs.update(
                samples_per_capture=fields['samples_per_capture'].value(),
                sample_min_period_s=fields['sample_min_period_s'].value(),
            )
        if 'pass_id' in fields:
            req_kwargs.update(
                pass_id=fields['pass_id'].value(),
                session_note=fields['session_note'].text(),
            )
        request = RunRequest(
            test=test,
            output_root=Path(DEFAULT_OUTPUT_DIR).expanduser(),
            initial_pose=(fields['initial_y'].value(), fields['initial_z'].value()),
            goals=tuple(goals),
            detection_max_age_s=fields['fresh_window'].value(),
            detection_timeout_s=fields['det_timeout'].value(),
            **req_kwargs,
        )
        if self._runner.request_run(request):
            self._start_btn.setEnabled(False)
            self._continue_btn.setEnabled(False)
            self._progress.setRange(0, test.total_visits())
            self._progress.setValue(0)
            self._detection_label.setText('detection: idle')
            self._banner.setVisible(False)
            self._log_msg(
                f'requested run: {test.name} '
                f'({test.num_cycles} cycles x {len(goals)} goals)')

    @staticmethod
    def _parse_goals_text(text: str) -> list:
        """Parse the multi-line goals editor into a list of (y, z) tuples.

        Each non-blank line must look like ``y, z``. Whitespace is
        ignored. Lines starting with '#' are treated as comments.
        Raises ValueError on any malformed line.
        """
        goals: list = []
        for lineno, raw in enumerate(text.splitlines(), start=1):
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 2:
                raise ValueError(
                    f'line {lineno}: expected "y, z", got {raw!r}')
            try:
                y, z = float(parts[0]), float(parts[1])
            except ValueError:
                raise ValueError(
                    f'line {lineno}: y or z is not a number ({raw!r})')
            goals.append((y, z))
        return goals

    def _update_grid_candidates(self, test_name: str):
        """IK-free candidate count shown live while the operator edits
        the rectangle; Generate replaces it with the filtered numbers."""
        fields = self._pages_fields.get(test_name, {})
        label = fields.get('grid_estimate')
        if label is None:
            return
        y0, y1 = fields['grid_y0'].value(), fields['grid_y1'].value()
        z0, z1 = fields['grid_z0'].value(), fields['grid_z1'].value()
        sp = fields['grid_spacing'].value()
        if y1 <= y0 or z1 <= z0 or sp <= 0:
            label.setText('empty rectangle')
            return
        n = ((int(round((y1 - y0) / sp)) + 1)
             * (int(round((z1 - z0) / sp)) + 1))
        label.setText(f'{n} candidate points - Generate to filter')

    def _on_generate_grid(self, test_name: str):
        """Fill the goals editor with the filtered serpentine grid.

        The comment header records the rectangle / limit / rejection
        provenance; _parse_goals_text skips '#' lines, and the goals
        themselves land in the run's config.yaml, so a sweep's grid
        parameters are always reconstructible from its run dir.
        """
        from ..grid import serpentine, filter_grid
        fields = self._pages_fields[test_name]
        y0, y1 = fields['grid_y0'].value(), fields['grid_y1'].value()
        z0, z1 = fields['grid_z0'].value(), fields['grid_z1'].value()
        spacing = fields['grid_spacing'].value()
        limit = fields['joint_limit_rad'].value()
        limit_min = fields['joint_limit_min_rad'].value()
        margin = fields['limit_margin_rad'].value()
        closure = fields['closure_margin_m'].value()
        if y1 <= y0 or z1 <= z0:
            self._log_msg('grid: empty rectangle (need y1 > y0 and z1 > z0)')
            return
        pts = serpentine(y0, y1, z0, z1, spacing)
        kept, stats = filter_grid(pts, limit, margin, closure,
                                  joint_limit_min_rad=limit_min)
        counts = (f'{stats.total} candidates -> {stats.kept} kept '
                  f'(ik {stats.ik_invalid}, limit {stats.joint_limit}, '
                  f'closure {stats.closure_margin} rejected)')
        header = (
            f'# grid rect y[{y0:.3f}, {y1:.3f}] z[{z0:.3f}, {z1:.3f}] '
            f'spacing {spacing:.3f}\n'
            f'# joint limits [{limit_min:.3f}, {limit:.3f}] rad '
            f'(margin {margin:.3f}), '
            f'closure margin {closure:.3f} m\n'
            f'# {counts}\n')
        fields['goals_edit'].setPlainText(
            header + ''.join(f'{y:.3f}, {z:.3f}\n' for y, z in kept))
        est_min = stats.kept * 13 / 60
        self._log_msg(f'grid: {counts}; ~{est_min:.0f} min per pass '
                      f'at 13 s/point')
        if 'grid_estimate' in fields:
            fields['grid_estimate'].setText(
                f'kept {stats.kept} / {stats.total} candidates, '
                f'~{est_min:.0f} min per pass')
        if not kept:
            self._log_msg('grid: nothing kept - check the rectangle and '
                          'the joint limit')

    def _on_load_anchors(self, test_name: str):
        """Populate the anchor combo from the sweep page's rectangle.

        Each anchor is reachability-checked independently (its own run
        seeded from home), NOT seed-chained across the large jumps
        between anchors.
        """
        from ..grid import nine_points, filter_grid
        fields = self._pages_fields[test_name]
        sweep = self._pages_fields.get('workspace_coverage', {})
        if 'grid_y0' not in sweep:
            self._log_msg('anchors: sweep-page grid fields unavailable')
            return
        y0, y1 = sweep['grid_y0'].value(), sweep['grid_y1'].value()
        z0, z1 = sweep['grid_z0'].value(), sweep['grid_z1'].value()
        anchors = nine_points(y0, y1, z0, z1,
                              inset=sweep['grid_spacing'].value())
        names = ['corner --', 'corner +-', 'corner -+', 'corner ++',
                 'mid bottom', 'mid top', 'mid left', 'mid right',
                 'center']
        combo = fields['anchor_combo']
        combo.clear()
        n_ok = 0
        for name, (y, z) in zip(names, anchors):
            ok = bool(filter_grid(
                [(y, z)], sweep['joint_limit_rad'].value(),
                sweep['limit_margin_rad'].value(),
                sweep['closure_margin_m'].value(),
                joint_limit_min_rad=sweep['joint_limit_min_rad'].value())[0])
            suffix = '' if ok else '  [unreachable]'
            combo.addItem(f'{name}  ({y:+.3f}, {z:.3f}){suffix}',
                          (y, z, ok))
            n_ok += ok
        self._log_msg(f'anchors: 9 points from the sweep rectangle '
                      f'y[{y0:.3f}, {y1:.3f}] z[{z0:.3f}, {z1:.3f}], '
                      f'{n_ok} reachable')

    def _on_anchor_selected(self, test_name: str):
        fields = self._pages_fields[test_name]
        data = fields['anchor_combo'].currentData()
        if not data:
            return
        y, z, ok = data
        if not ok:
            self._log_msg(f'anchor ({y:.3f}, {z:.3f}) is outside the '
                          f'filtered workspace; pick another or adjust '
                          f'the rectangle')
        fields['goal_y'].setValue(y)
        fields['goal_z'].setValue(z)

    @Slot()
    def _on_continue_clicked(self):
        self._awaiting_continue = False
        self._auto_continue_fresh_since = None
        self._runner.proceed()
        self._continue_btn.setEnabled(False)

    @Slot()
    def _on_cancel_clicked(self):
        # Stop in place; do not return the arm anywhere. Reset is the
        # button to use when you want the arm parked at the typed
        # initial pose after halting. Cancel is wired to both runners
        # so it works whether a test or a calibration sweep is active
        # (only one of them runs at a time, but cancelling the idle
        # one is a no-op).
        self._awaiting_continue = False
        self._auto_continue_fresh_since = None
        self._runner.cancel()
        self._cam_runner.cancel()
        self._reset_ui_state(status='cancelled')

    @Slot()
    def _on_reset_clicked(self):
        fields = self._active_fields()
        if fields is None:
            self._log_msg('select a test page to reset the arm to its initial')
            return
        self._runner.reset_to(fields['initial_y'].value(),
                              fields['initial_z'].value())
        self._reset_ui_state(status='resetting: returning arm to initial')

    # -- homing (Start tab) --------------------------------------

    @Slot()
    def _on_home_clicked(self):
        if not self._runner.home():
            return
        self._home_btn.setEnabled(False)
        self._home_status.setText('homing: seeking limit switches...')
        self._home_status.setStyleSheet('color: #c79a3a;')

    @Slot(bool, str)
    def _on_home_finished(self, ok: bool, message: str):
        self._home_btn.setEnabled(True)
        if ok:
            self._home_status.setText('homed')
            self._home_status.setStyleSheet('color: #2e9c4a;')
        else:
            self._home_status.setText(
                f'home failed: {message}' if message else 'home failed')
            self._home_status.setStyleSheet('color: #d04b4b;')

    # -- camera localization -------------------------------------

    def _refresh_alignment_state(self):
        """Refresh the URDF-detected mode + the alignment status label,
        and gate the Calibrate-camera button on a recognised mode.

        Pure TF + file-system probe; cheap to run on a 2 s timer. The
        "in progress" label is driven by the runner's status_cb.
        """
        # Mode comes from the URDF (parent of camera_link). When the
        # URDF isn't loaded yet (just after launch), this returns None
        # transiently -- the timer retries and we settle within ~2 s.
        mode = self._cam_runner.detect_mode()
        if mode == MODE_STAND:
            self._mode_label.setText('mode: camera on stand (calibration_stand)')
            self._mode_label.setStyleSheet('color: #2e9c4a;')
            mode_ok = True
        elif mode == MODE_ON_ROBOT:
            self._mode_label.setText('mode: camera on robot (on_robot_mount)')
            self._mode_label.setStyleSheet('color: #2e9c4a;')
            mode_ok = True
        else:
            self._mode_label.setText(
                'mode: URDF not in calibration-capable configuration')
            self._mode_label.setStyleSheet('color: #d04b4b;')
            mode_ok = False

        if self._cam_runner.is_busy():
            self._align_status.setText('localization: in progress')
            self._align_status.setStyleSheet('color: #c79a3a;')
            self._calibrate_btn.setEnabled(False)
            return
        latest = self._latest_result_yaml()
        applied = self._camera_pose_config_path().exists()
        applied_str = ' (applied at launch)' if applied else ''
        if latest is not None:
            self._align_status.setText(
                f'localization: last result {latest.parent.name} '
                f'({latest.parent.parent.name}){applied_str}')
            self._align_status.setStyleSheet('color: #2e9c4a;')
        elif applied:
            # Edge case: config file exists but no per-run result.yaml
            # (e.g. operator hand-edited the config, or wiped data/).
            self._align_status.setText(
                f'localization: no run yet, config applied at launch')
            self._align_status.setStyleSheet('color: #2e9c4a;')
        else:
            self._align_status.setText('localization: not run yet')
            self._align_status.setStyleSheet('color: gray;')
        self._calibrate_btn.setEnabled(mode_ok)

    def _latest_result_yaml(self) -> Optional[Path]:
        root = (Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
                     'data/camera_localization').expanduser())
        if not root.exists():
            return None
        candidates = sorted(root.glob('*/*/result.yaml'))
        return candidates[-1] if candidates else None

    def _camera_pose_config_path(self) -> Path:
        return Path(
            '~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
            'config/camera_pose.yaml').expanduser()

    @Slot()
    def _on_calibrate_clicked(self):
        if self._cam_runner.is_busy():
            return
        if self._cam_runner.request():
            self._log_msg('camera localization: starting')
        self._refresh_alignment_state()

    @Slot(str, str, str)
    def _on_camera_calib_finished(self, status: str, result_path: str,
                                  reason: str):
        """Slot for CameraCalibrationRunner.finished_cb.

        Just logs the outcome and refreshes the alignment status; no
        TF publisher to start (the URDF chain stays the source of
        truth).
        """
        if status == 'completed':
            self._log_msg(f'camera localization saved: {result_path}')
            self._status_label.setText('camera localization completed')
        elif status == 'canceled':
            self._log_msg('camera localization canceled')
            self._status_label.setText('camera localization canceled')
        else:
            text = (f'camera localization failed: {reason}' if reason
                    else 'camera localization failed')
            self._log_msg(text)
            self._status_label.setText(text)
        self._refresh_alignment_state()

    # -- runner-side slots ---------------------------------------

    @Slot(str)
    def _on_status(self, msg: str):
        self._status_label.setText(msg)
        self._log_msg(msg)

    @Slot(int, int)
    def _on_progress(self, current: int, total: int):
        self._progress.setRange(0, max(total, 1))
        self._progress.setValue(current)

    @Slot(int, int)
    def _on_awaiting_continue(self, iteration: int, total: int):
        self._awaiting_continue = True
        self._auto_continue_fresh_since = None
        if self._auto_continue.isChecked():
            self._status_label.setText(
                f'iteration {iteration}/{total}: '
                f'auto-continue (waiting for fresh detection)')
        else:
            self._status_label.setText(
                f'iteration {iteration}/{total}: '
                f'position camera, then click Continue')

    @Slot(bool, float)
    def _on_detection_state(self, is_fresh: bool, age_s: float):
        self._continue_btn.setEnabled(is_fresh)
        if is_fresh:
            self._detection_label.setText(
                f'detection: fresh ({age_s * 1000:.0f} ms old)')
            self._detection_label.setStyleSheet('color: green;')
        else:
            self._detection_label.setText('detection: not visible')
            self._detection_label.setStyleSheet('color: red;')
        self._maybe_auto_continue(is_fresh)

    def _maybe_auto_continue(self, is_fresh: bool):
        """Advance the run automatically once detection has been fresh
        for ``auto_hold`` seconds at the current Continue gate.

        Hysteresis: a single fresh frame won't trip auto-advance; the
        detection must stay fresh continuously for the hold time. Any
        loss of freshness resets the timer. Cancel always wins because
        it sets ``_stop_event`` in the runner, which causes
        ``_wait_for_continue`` to exit before our proceed() lands.
        """
        if not (self._auto_continue.isChecked() and self._awaiting_continue):
            return
        if not is_fresh:
            self._auto_continue_fresh_since = None
            return
        now = time.monotonic()
        if self._auto_continue_fresh_since is None:
            self._auto_continue_fresh_since = now
            return
        if now - self._auto_continue_fresh_since >= self._auto_hold.value():
            self._auto_continue_fresh_since = None
            self._on_continue_clicked()

    @Slot(str, str)
    def _on_finished(self, run_dir: str, status: str):
        self._awaiting_continue = False
        self._auto_continue_fresh_since = None
        self._log_msg(f'output: {run_dir}')
        self._reset_ui_state(status=f'run {status}')
        self._refresh_run_counts()
        self._show_banner(run_dir, status)

    # Multi-goal tests resume at visit granularity (a NEW run dir with
    # skip_visits; the analysis pools the pass pieces by pass_id).
    # Backlash is excluded: its approach offset is a ctor arg not
    # recorded in config.yaml, so the visit list can't be reconstructed
    # faithfully. Single-goal tests keep the cycle-level append resume.
    _VISIT_RESUME_TESTS = ('workspace_coverage', 'noise_gate',
                           'settle_probe')

    def _resume_info(self, run_dir: Optional[Path]):
        """(config, resume_state) for an interrupted run dir, or None
        when the run cannot be resumed (missing config, unknown test,
        or nothing left to capture). resume_state['mode'] is 'visits'
        (sweep-style, new dir + skip_visits) or 'cycles' (single-goal,
        append to the same dir)."""
        if run_dir is None or not (Path(run_dir) / 'config.yaml').exists():
            return None
        try:
            with (Path(run_dir) / 'config.yaml').open() as f:
                cfg = yaml.safe_load(f) or {}
            goals = [tuple(g) for g in (cfg.get('goals') or [])]
            num_cycles = int(cfg.get('num_cycles') or 0)
            test_name = cfg.get('test_name')
            if test_name not in TEST_REGISTRY or not goals \
                    or num_cycles < 1:
                return None
            if test_name in self._VISIT_RESUME_TESTS:
                test = TEST_REGISTRY[test_name](
                    targets=goals, num_cycles=num_cycles,
                    settle_time=float(cfg.get('settle_time', 2.0)),
                    return_home_between_targets=True)
                state = load_sweep_resume_state(
                    run_dir, test.total_visits())
                if not state['resumable']:
                    return None
                state['mode'] = 'visits'
                state['total_visits'] = test.total_visits()
                return cfg, state
            state = load_resume_state(run_dir, len(goals), num_cycles)
            if not state['resumable']:
                return None
            state['mode'] = 'cycles'
            return cfg, state
        except Exception:  # noqa: BLE001
            return None

    @Slot()
    def _on_banner_resume(self):
        info = self._resume_info(self._last_run_dir)
        if info is None:
            self._log_msg('cannot resume: run is not resumable')
            return
        cfg, state = info
        test_name = cfg['test_name']
        goals = [tuple(g) for g in cfg['goals']]
        num_cycles = int(cfg['num_cycles'])
        cls = TEST_REGISTRY[test_name]
        extra = {}
        if cfg.get('verify_home_with_tag'):
            extra['verify_home_with_tag'] = True
        try:
            test = cls(
                targets=goals,
                num_cycles=num_cycles,
                settle_time=float(cfg.get('settle_time', 2.0)),
                return_home_between_targets=True,
                **extra,
            )
        except ValueError as exc:
            self._log_msg(f'cannot resume: {exc}')
            return
        # Everything comes from the interrupted run's config, not the
        # page widgets, so the resumed visits are captured under
        # identical conditions to the originals.
        req_kwargs = dict(
            test=test,
            output_root=Path(DEFAULT_OUTPUT_DIR).expanduser(),
            initial_pose=tuple(cfg.get('initial_pose', (0.0, 0.5))),
            goals=tuple(goals),
            detection_max_age_s=float(cfg.get('detection_max_age_s', 0.5)),
            detection_timeout_s=float(cfg.get('detection_timeout_s', 5.0)),
            home_tol_m=float(cfg.get('home_tol_m', 0.02)),
            home_hold_frames=int(cfg.get('home_hold_frames', 5)),
            home_timeout_s=float(cfg.get('home_timeout_s', 10.0)),
            samples_per_capture=int(cfg.get('samples_per_capture', 1)),
            sample_min_period_s=float(cfg.get('sample_min_period_s', 0.0)),
            pass_id=int(cfg.get('pass_id', 1)),
            session_note=str(cfg.get('session_note', '')),
        )
        if state['mode'] == 'visits':
            # Sweep-style resume: NEW run dir, skip the visits already
            # captured. Keeps the interrupted run's pass_id so the
            # analysis reassembles the pieces into one pass.
            req_kwargs['skip_visits'] = state['skip_visits']
            done, total = state['skip_visits'], state['total_visits']
        else:
            req_kwargs['resume_dir'] = Path(self._last_run_dir)
            req_kwargs['start_cycle'] = state['next_cycle']
            done, total = state['done_visits'], test.total_visits()
        request = RunRequest(**req_kwargs)
        if self._runner.request_run(request):
            self._banner.setVisible(False)
            self._start_btn.setEnabled(False)
            self._continue_btn.setEnabled(False)
            self._progress.setRange(0, total)
            self._progress.setValue(done)
            if state['mode'] == 'visits':
                self._log_msg(
                    f"resuming {cfg.get('run_id')} in a new run dir: "
                    f"skipping {done}/{total} captured visits "
                    f"(pass {req_kwargs['pass_id']})")
            else:
                self._log_msg(
                    f"resuming {cfg.get('run_id')} from cycle "
                    f"{state['next_cycle']}/{num_cycles}")

    def _show_banner(self, run_dir: str, status: str):
        self._last_run_dir = Path(run_dir) if run_dir else None
        self._last_test_name = self._current_test_name()
        colour, label = {
            'completed': ('#2e9c4a', 'COMPLETED'),
            'canceled':  ('#c79a3a', 'CANCELED'),
            'failed':    ('#d04b4b', 'FAILED'),
        }.get(status, ('#888888', status.upper()))
        self._banner_status.setText(f'Run {label}')
        self._banner_status.setStyleSheet(
            f'font-weight: bold; padding: 4px; '
            f'color: white; background-color: {colour};')
        if self._last_run_dir is not None:
            self._banner_path.setText(str(self._last_run_dir))
        else:
            self._banner_path.setText('(no run directory)')
        self._banner_delete.setEnabled(self._last_run_dir is not None
                                       and self._last_run_dir.exists())
        # Resume covers both interruption flavours: 'failed' (detection
        # loss etc.) and 'canceled' (operator stop / Ctrl+C) -- an
        # interrupted sweep is normally 'canceled' and its captured
        # visits are valid data.
        info = (self._resume_info(self._last_run_dir)
                if status in ('failed', 'canceled') else None)
        self._banner_resume.setEnabled(info is not None)
        if info is not None and info[1]['mode'] == 'visits':
            self._banner_resume.setToolTip(
                'Start a new run covering the remaining visits '
                '(same pass id; the analysis pools the pieces).')
        elif info is not None:
            self._banner_resume.setToolTip(
                'Continue this run from its first incomplete cycle, '
                'appending to the same data files.')
        else:
            self._banner_resume.setToolTip(
                'Only interrupted runs with remaining work can be '
                'resumed.')
        self._banner_open.setEnabled(self._last_run_dir is not None
                                     and self._last_run_dir.exists())
        self._banner.setVisible(True)

    @Slot()
    def _on_banner_keep(self):
        self._banner.setVisible(False)

    @Slot()
    def _on_banner_delete(self):
        if self._last_run_dir is None or not self._last_run_dir.exists():
            return
        reply = QMessageBox.question(
            self, 'Delete run',
            f'Permanently delete this run directory?\n\n{self._last_run_dir}',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            shutil.rmtree(self._last_run_dir)
            self._log_msg(f'deleted: {self._last_run_dir}')
            self._banner.setVisible(False)
        except OSError as exc:
            self._log_msg(f'delete failed: {exc}')

    @Slot()
    def _on_banner_open_folder(self):
        # Open the run directory in the file manager so the operator can
        # eyeball the CSVs before evaluating in the test's notebook
        # (experiments/notebooks/<test>.ipynb).
        if self._last_run_dir is None or not self._last_run_dir.exists():
            return
        try:
            subprocess.Popen(['xdg-open', str(self._last_run_dir)])
        except OSError as exc:
            self._log_msg(f'open folder failed: {exc}')

    def shutdown(self):
        self._align_timer.stop()
        self._cam_runner.shutdown()
        self._runner.shutdown()

    # Widget keys excluded from persistence. 'iterations' always opens
    # at the per-test protocol default (30 / 30 / 3) on a fresh GUI so
    # the recommended count is never silently overridden by whatever a
    # previous session happened to use; the operator can still change
    # it for the session.
    _UNPERSISTED_KEYS = ('iterations',)

    def save_settings(self, plugin_settings):
        # Note: the active sidebar page is intentionally NOT persisted; the
        # GUI always opens on the Start tab (see restore_settings).
        # Persist only the input widgets in each tab's bundle; skip the
        # run-control buttons / labels / progress bar (transient state).
        for test_name, fields in self._pages_fields.items():
            for key, widget in fields.items():
                if key in self._UNPERSISTED_KEYS:
                    continue
                if isinstance(widget, QCheckBox):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.isChecked())
                elif isinstance(widget, QPlainTextEdit):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.toPlainText())
                elif isinstance(widget, QLineEdit):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.text())
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.value())

    def restore_settings(self, plugin_settings):
        for test_name, fields in self._pages_fields.items():
            for key, widget in fields.items():
                if key in self._UNPERSISTED_KEYS:
                    continue
                v = plugin_settings.value(f'{test_name}/{key}')
                if v is None:
                    continue
                if isinstance(widget, QCheckBox):
                    widget.setChecked(str(v).lower() in ('1', 'true'))
                elif isinstance(widget, QPlainTextEdit):
                    widget.setPlainText(str(v))
                elif isinstance(widget, QLineEdit):
                    widget.setText(str(v))
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    try:
                        widget.setValue(type(widget.value())(v))
                    except (TypeError, ValueError):
                        pass
        # A measured joint limit always wins over whatever spinbox value
        # the last session persisted.
        self._apply_measured_joint_limit()
        # Always open on the Start tab, regardless of the last session's
        # page. The sidebar is already built at _PAGE_START; we just make
        # the intent explicit and don't restore any saved active page.
        self._nav.setCurrentRow(self._PAGE_START)
