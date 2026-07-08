"""Qt widget for the calibration dashboard.

MoveIt-Setup-Assistant-style layout: a left sidebar picks a step (Start,
Camera Localization, or one of the accuracy/repeatability/workspace tests)
and the right panel swaps to that step's controls. The Start tab only
offers robot homing; each test tab is self-contained (its params, capture
settings, and Start / Continue / Reset / Cancel). A shared strip beneath
the pages holds the status line, log, and post-run result banner.

Workflow (real hardware only):
  1. Terminal 1: bring up the robot with the AprilTag detector
     (`real_bringup.launch.py mode:=tests calibration:=true`).
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
    QCheckBox, QDoubleSpinBox, QFrame, QMessageBox, QSizePolicy,
    QSpinBox, QPushButton, QLabel, QListWidget, QListWidgetItem,
    QStackedWidget, QPlainTextEdit, QProgressBar, QTextEdit,
)


# Status messages matching any of these patterns (case-insensitive)
# get logged in red. Catches IK / motion / settle failures, aborts,
# missing services, stale detections, etc.
_ERROR_PATTERNS = re.compile(
    r'(fail|abort|cannot|invalid|stale|out of reach|error|empty|no goals'
    r'|not visible|not moving)', re.IGNORECASE)
# Successes (completions, captures, arrivals) get logged in green so
# the operator can scan progress quickly. The match is intentionally
# narrow to avoid colouring routine progress lines.
_SUCCESS_PATTERNS = re.compile(
    r'(\bcompleted\b|\barrived\b|\bcaptured\b|run completed)',
    re.IGNORECASE)

from ..runner import (
    CalibrationRunner, CameraCalibrationRunner, RunRequest, TEST_REGISTRY,
    MODE_STAND, MODE_ON_ROBOT,
)


DEFAULT_OUTPUT_DIR = '~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data'

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

    # Sidebar rows / stacked-page indices. The three test pages map to a
    # TEST_REGISTRY key; Start and Camera pages have no test.
    _PAGE_START = 0
    _PAGE_CAMERA = 1
    _PAGE_STATIC = 2
    _PAGE_REPEAT = 3
    _PAGE_WORKSPACE = 4
    _PAGE_TEST_NAME = {
        _PAGE_STATIC: 'static_accuracy',
        _PAGE_REPEAT: 'repeatability',
        _PAGE_WORKSPACE: 'workspace_coverage',
    }
    _NAV_LABELS = (
        'Start', 'Camera Localization',
        'Static Accuracy', 'Repeatability', 'Workspace Coverage',
    )

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

        right = QVBoxLayout()
        self._pages = QStackedWidget()
        self._pages.addWidget(self._build_start_page())
        self._pages.addWidget(self._build_camera_page())
        self._pages.addWidget(self._build_test_page(
            'static_accuracy', with_iterations=True,
            with_home_gate=False, goal_mode='single'))
        self._pages.addWidget(self._build_test_page(
            'repeatability', with_iterations=True,
            with_home_gate=True, goal_mode='single'))
        self._pages.addWidget(self._build_test_page(
            'workspace_coverage', with_iterations=False,
            with_home_gate=False, goal_mode='list'))
        # Keep the page compact (sized to its content) and let the run
        # panel's log expand to fill the rest, so there's no large blank
        # gap between a page's controls and the log at the bottom.
        self._pages.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        right.addWidget(self._pages)
        right.addWidget(self._build_run_panel(), stretch=1)
        root.addLayout(right, stretch=1)

        # The run-control widgets (Start/Continue/Reset/Cancel, capture
        # settings, detection, progress) now live per test tab. Bind the
        # self._* names used by the runner-callback slots to a default tab
        # so a stray callback before the first tab switch is harmless;
        # _on_page_changed rebinds them to whichever test tab is active.
        self._bind_run_widgets(self._pages_fields['static_accuracy'])
        self._apply_styles()

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
        page = QWidget()
        v = QVBoxLayout(page)
        title = QLabel('Volcaniarm Calibration')
        title.setStyleSheet('font-size: 22px; font-weight: bold;')
        v.addWidget(title)

        pixmap = self._load_logo_pixmap()
        image = QLabel()
        image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if pixmap is not None:
            image.setPixmap(pixmap)
        else:
            image.setText('(robot image unavailable)')
            image.setStyleSheet('color: gray;')
        v.addWidget(image)

        instructions = QLabel(
            '<p>Calibrate the real Volcaniarm against AprilTag ground truth.</p>'
            '<p><b>Launch order</b></p>'
            '<ol>'
            '<li>Terminal 1 - robot + camera + AprilTag detector + RViz:<br>'
            '<code>ros2 launch volcaniarm_bringup real_bringup.launch.py '
            'mode:=tests calibration:=true</code></li>'
            '<li>Terminal 2 - this GUI:<br>'
            '<code>ros2 launch volcaniarm_calibration calibration_gui.launch.py</code>'
            '</li>'
            '</ol>'
            '<p><b>Steps (left sidebar)</b></p>'
            '<ul>'
            '<li><b>Camera Localization</b> - measure where the camera '
            'sits relative to the arm base before running tests.</li>'
            '<li><b>Static Accuracy</b> - one goal, N cycles, returning '
            'to the initial pose each visit; reports the per-visit error '
            'distribution.</li>'
            '<li><b>Repeatability</b> - one goal, N cycles, gated on a '
            'tag-confirmed home between iterations (ISO 9283 RP_yz).</li>'
            '<li><b>Workspace Coverage</b> - sweep a list of goals once '
            'each across the envelope.</li>'
            '</ul>')
        instructions.setWordWrap(True)
        instructions.setTextFormat(Qt.TextFormat.RichText)
        v.addWidget(instructions)

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
            'Run limit-switch homing (volcaniarm_hardware/home). The arm '
            'seeks its limit switches and re-zeros; takes up to ~30 s. '
            'Use this if the robot booted with auto_home:=false.')
        home_outer.addWidget(self._home_btn)
        self._home_status = QLabel('not homed this session')
        self._home_status.setStyleSheet('color: gray;')
        home_outer.addWidget(self._home_status)
        v.addWidget(home_box)
        return page

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
                         with_home_gate: bool, goal_mode: str) -> QWidget:
        """Build one accuracy/repeatability/workspace page.

        Each page owns its own pose/goal/iterations/home widgets (a Qt
        widget can only live in one layout, so they can't be shared across
        pages). The shared capture settings (settle/fresh/auto) live in the
        run panel instead. Widget references are stashed in
        ``self._pages_fields[test_name]`` for the run/reset/seed paths.
        """
        page = QWidget()
        v = QVBoxLayout(page)
        fields: dict = {}

        if with_iterations:
            cfg_box = QGroupBox('Test configuration')
            cfg_form = QFormLayout(cfg_box)
            iterations = QSpinBox()
            iterations.setRange(1, 100)
            iterations.setValue(3)
            cfg_form.addRow('iterations', iterations)
            fields['iterations'] = iterations
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

        if goal_mode == 'single':
            goal_box = QGroupBox('Goal pose (workspace, metres)')
            goal_form = QFormLayout(goal_box)
            goal_y = self._make_pose_spinbox(_HOME_FALLBACK[0], lo=-0.4, hi=0.4)
            goal_form.addRow('y', goal_y)
            goal_z = self._make_pose_spinbox(_HOME_FALLBACK[1], lo=0.1, hi=0.9)
            goal_form.addRow('z', goal_z)
            fields['goal_y'] = goal_y
            fields['goal_z'] = goal_z
            v.addWidget(goal_box)
        else:
            goals_box = QGroupBox(
                'Goals list (workspace, metres) - one "y, z" per line')
            goals_outer = QVBoxLayout(goals_box)
            goals_edit = QPlainTextEdit()
            goals_edit.setPlaceholderText(
                '0.0, 0.5\n0.1, 0.5\n-0.1, 0.5\n0.0, 0.6')
            goals_edit.setMaximumBlockCount(200)
            goals_outer.addWidget(goals_edit)
            fields['goals_edit'] = goals_edit
            v.addWidget(goals_box)

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
        settle_time.setValue(2.0)
        cap_form.addRow('settle time (s)', settle_time)
        fresh_window = QDoubleSpinBox()
        fresh_window.setRange(0.1, 2.0)
        fresh_window.setSingleStep(0.1)
        fresh_window.setDecimals(2)
        fresh_window.setValue(0.5)
        cap_form.addRow('detection fresh window (s)', fresh_window)
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
        fields['auto_continue'] = auto_continue
        fields['auto_hold'] = auto_hold
        v.addWidget(cap_box)

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
        self._banner_delete = QPushButton('Delete run')
        self._banner_open = QPushButton('Open notebook')
        self._banner_keep.clicked.connect(self._on_banner_keep)
        self._banner_delete.clicked.connect(self._on_banner_delete)
        self._banner_open.clicked.connect(self._on_banner_open_notebook)
        btn_row.addWidget(self._banner_keep)
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
        if test_name == 'workspace_coverage':
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
        num_cycles = fields['iterations'].value() if 'iterations' in fields else 1
        # Test classes still take a `targets` list (kept for backward
        # compat with iter_visits); the runner reads `request.goals`.
        extra = {}
        if 'verify_home' in fields:  # repeatability page: opt-in home gate
            extra['verify_home_with_tag'] = fields['verify_home'].isChecked()
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
        # Home-confirm params only exist on the repeatability page; the
        # RunRequest dataclass supplies sensible defaults otherwise.
        home_kwargs = {}
        if 'home_tol_mm' in fields:
            home_kwargs = dict(
                home_tol_m=fields['home_tol_mm'].value() / 1000.0,
                home_hold_frames=fields['home_hold_frames'].value(),
                home_timeout_s=fields['home_timeout_s'].value(),
            )
        request = RunRequest(
            test=test,
            output_root=Path(DEFAULT_OUTPUT_DIR).expanduser(),
            initial_pose=(fields['initial_y'].value(), fields['initial_z'].value()),
            goals=tuple(goals),
            detection_max_age_s=fields['fresh_window'].value(),
            **home_kwargs,
        )
        if self._runner.request_run(request):
            self._start_btn.setEnabled(False)
            self._continue_btn.setEnabled(False)
            self._progress.setRange(0, test.num_cycles * len(goals))
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
        self._show_banner(run_dir, status)

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
        self._banner_open.setEnabled(self._notebook_path() is not None)
        self._banner_open.setToolTip(
            '' if self._notebook_path() is not None
            else 'notebook for this test type does not exist yet')
        self._banner.setVisible(True)

    def _notebook_path(self) -> Optional[Path]:
        """Resolve the analysis notebook path for the current test type.

        Prefers the source tree (the git-tracked notebook, so re-running
        it and letting nbstripout keep it clean edits the file you'd
        actually commit), then falls back to the installed share dir for a
        production install with no source tree. Returns None if neither
        exists, so the Open button can disable itself.
        """
        if not self._last_test_name:
            return None
        candidates: list = []
        # Source tree first. The widget lives at
        # <pkg>/volcaniarm_calibration/rqt/calibration_dashboard_widget.py;
        # the notebooks dir is two levels up under the package root.
        candidates.append(Path(__file__).resolve().parents[2]
                          / 'notebooks' / f'{self._last_test_name}.ipynb')
        try:
            share = Path(get_package_share_directory('volcaniarm_calibration'))
            candidates.append(share / 'notebooks' / f'{self._last_test_name}.ipynb')
        except Exception:
            pass
        for path in candidates:
            if path.exists():
                return path
        return None

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
    def _on_banner_open_notebook(self):
        # Open the analysis notebook directly. Re-running it is safe for
        # git: the repo's nbstripout filter (*.ipynb filter=nbstripout in
        # .gitattributes) strips cell outputs / execution counts, so the
        # regenerated graphs never show up as a working-tree change.
        path = self._notebook_path()
        if path is None:
            return
        try:
            subprocess.Popen(['xdg-open', str(path)])
        except OSError as exc:
            self._log_msg(f'open notebook failed: {exc}')

    def shutdown(self):
        self._align_timer.stop()
        self._cam_runner.shutdown()
        self._runner.shutdown()

    def save_settings(self, plugin_settings):
        # Note: the active sidebar page is intentionally NOT persisted; the
        # GUI always opens on the Start tab (see restore_settings).
        # Persist only the input widgets in each tab's bundle; skip the
        # run-control buttons / labels / progress bar (transient state).
        for test_name, fields in self._pages_fields.items():
            for key, widget in fields.items():
                if isinstance(widget, QCheckBox):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.isChecked())
                elif isinstance(widget, QPlainTextEdit):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.toPlainText())
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    plugin_settings.set_value(
                        f'{test_name}/{key}', widget.value())

    def restore_settings(self, plugin_settings):
        for test_name, fields in self._pages_fields.items():
            for key, widget in fields.items():
                v = plugin_settings.value(f'{test_name}/{key}')
                if v is None:
                    continue
                if isinstance(widget, QCheckBox):
                    widget.setChecked(str(v).lower() in ('1', 'true'))
                elif isinstance(widget, QPlainTextEdit):
                    widget.setPlainText(str(v))
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    try:
                        widget.setValue(type(widget.value())(v))
                    except (TypeError, ValueError):
                        pass
        # Always open on the Start tab, regardless of the last session's
        # page. The sidebar is already built at _PAGE_START; we just make
        # the intent explicit and don't restore any saved active page.
        self._nav.setCurrentRow(self._PAGE_START)
