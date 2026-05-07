"""Qt widget for the calibration dashboard.

Hands-off (auto-continue) and operator-gated flows for real-hardware
calibration:

  1. Operator picks the test type, fills in initial and goal pose
     (workspace y, z metres), iteration count, and (for repeatability)
     the home-confirm gate parameters.
  2. Click Start Run. The arm moves to the initial pose, captures a
     baseline reading of the apriltag base->ee transform, then begins
     the iteration loop.
  3. At each iteration the arm moves to the goal, settles, and the
     runner waits for a freshly progressed apriltag TF stamp before
     capturing. Auto-continue advances on the next fresh detection;
     unchecking it falls back to a manual Continue click. Cancel
     aborts the whole run.
  4. Repeatability tests additionally gate each return-to-home on the
     detected vs URDF Y-Z segment length agreeing within tolerance for
     the configured number of consecutive fresh frames.

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

from python_qt_binding.QtCore import Signal, Slot, QObject, QTimer
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QMessageBox,
    QSpinBox, QPushButton, QLabel,
    QPlainTextEdit, QProgressBar, QTextEdit,
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
    # Camera calibration completion. (status, calib_path_or_empty, reason)
    camera_calib_finished = Signal(str, str, str)


class CalibrationDashboardWidget(QWidget):

    def __init__(self, node):
        super().__init__()
        self.setObjectName('CalibrationDashboardWidget')
        self.setWindowTitle('Volcaniarm Calibration Dashboard')

        self._node = node
        # Dashboard-scope flag set by the launch when calibration:=true.
        # When True, the widget hides the test-runner controls and only
        # exposes the "Camera localization" group. The launch passes the
        # value as a PythonExpression-evaluated string ('True'/'False'),
        # so handle either str or bool here.
        try:
            if not node.has_parameter('camera_calibration_only'):
                node.declare_parameter('camera_calibration_only', False)
            raw = node.get_parameter('camera_calibration_only').value
            if isinstance(raw, str):
                self._camera_calibration_only = raw.strip().lower() == 'true'
            else:
                self._camera_calibration_only = bool(raw)
        except Exception:
            self._camera_calibration_only = False
        self._bridge = _RunnerBridge()
        self._runner = CalibrationRunner(node)
        self._runner.status_cb = self._bridge.status.emit
        self._runner.progress_cb = self._bridge.progress.emit
        self._runner.finished_cb = lambda p, s: self._bridge.finished.emit(str(p), s)
        self._runner.awaiting_continue_cb = self._bridge.awaiting_continue.emit
        self._runner.detection_state_cb = self._bridge.detection_state.emit

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

        # Camera-localization runner: derives camera-in-base from one
        # AprilTag detection and writes a YAML record. Composes with
        # the test runner -- shares its TF buffer and _stop_event so a
        # single Cancel halts whichever is active.
        self._cam_runner = CameraCalibrationRunner(self._runner)
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
        layout = QVBoxLayout(self)

        # Camera localization: measure where the camera is relative to
        # the arm base, using one detection of the base AprilTag. No
        # arm motion, no TF publish (the URDF chain remains the source
        # of truth). Operator uses this to compare measured-vs-URDF
        # before / after re-mounting the camera.
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
        self._calibrate_btn.setToolTip(
            'Sweeps the arm through the EE poses in calibration_poses.yaml '
            'and solves for the camera pose. Mode is auto-detected from '
            'the URDF: parent of camera_link is world (stand) or '
            'camera_mount_rev_link (on-robot).')
        align_outer.addWidget(self._calibrate_btn)
        layout.addWidget(align_box)

        cfg_box = QGroupBox('Test configuration')
        cfg_form = QFormLayout(cfg_box)
        self._test_combo = QComboBox()
        for name in TEST_REGISTRY.keys():
            self._test_combo.addItem(name)
        cfg_form.addRow('test type', self._test_combo)
        self._iterations = QSpinBox()
        self._iterations.setRange(1, 100)
        self._iterations.setValue(3)
        cfg_form.addRow('iterations', self._iterations)
        self._settle_time = QDoubleSpinBox()
        self._settle_time.setRange(0.0, 10.0)
        self._settle_time.setSingleStep(0.5)
        self._settle_time.setDecimals(1)
        self._settle_time.setValue(2.0)
        cfg_form.addRow('settle time (s)', self._settle_time)
        self._fresh_window = QDoubleSpinBox()
        self._fresh_window.setRange(0.1, 2.0)
        self._fresh_window.setSingleStep(0.1)
        self._fresh_window.setDecimals(2)
        self._fresh_window.setValue(0.5)
        cfg_form.addRow('detection fresh window (s)', self._fresh_window)
        # Auto-continue: when checked, the runner auto-advances at each
        # Continue gate once detection has been continuously fresh for
        # `fresh-hold` seconds. Cancel still aborts immediately. Hold
        # time provides hysteresis against single-frame fresh blips.
        self._auto_continue = QCheckBox('auto-continue when detection fresh')
        self._auto_continue.setChecked(True)
        cfg_form.addRow(self._auto_continue)
        self._auto_hold = QDoubleSpinBox()
        self._auto_hold.setRange(0.2, 5.0)
        self._auto_hold.setSingleStep(0.1)
        self._auto_hold.setDecimals(2)
        self._auto_hold.setValue(1.0)
        cfg_form.addRow('auto-continue fresh-hold (s)', self._auto_hold)
        layout.addWidget(cfg_box)

        # Home-confirm gate: only meaningful for tests that opt in via
        # `verify_home_with_tag` (currently the repeatability test).
        # Toggles visibility on test-type change so the operator only
        # sees the controls when they apply.
        self._home_box = QGroupBox('Home-confirm gate (repeatability)')
        home_form = QFormLayout(self._home_box)
        self._home_tol_mm = QDoubleSpinBox()
        self._home_tol_mm.setRange(1.0, 100.0)
        self._home_tol_mm.setSingleStep(1.0)
        self._home_tol_mm.setDecimals(1)
        self._home_tol_mm.setSuffix(' mm')
        self._home_tol_mm.setValue(20.0)
        home_form.addRow('Y-Z segment tol', self._home_tol_mm)
        self._home_hold_frames = QSpinBox()
        self._home_hold_frames.setRange(1, 30)
        self._home_hold_frames.setValue(5)
        home_form.addRow('hold (consecutive fresh frames)', self._home_hold_frames)
        self._home_timeout_s = QDoubleSpinBox()
        self._home_timeout_s.setRange(1.0, 60.0)
        self._home_timeout_s.setSingleStep(1.0)
        self._home_timeout_s.setDecimals(1)
        self._home_timeout_s.setSuffix(' s')
        self._home_timeout_s.setValue(10.0)
        home_form.addRow('timeout', self._home_timeout_s)
        layout.addWidget(self._home_box)
        # Default invisible; _on_test_changed flips it for repeatability.
        self._home_box.setVisible(False)

        # Initial pose (defaults to the workspace (y, z) of theta=(0,0)
        # once the FK service responds; until then a sentinel value).
        # Each row has a small button to snap that axis back to home FK
        # so the operator can quickly recover after experimenting.
        initial_box = QGroupBox('Initial pose (workspace, metres)')
        initial_outer = QVBoxLayout(initial_box)
        initial_form = QFormLayout()
        self._initial_y = self._make_pose_spinbox(_HOME_FALLBACK[0],
                                                  lo=-0.4, hi=0.4)
        self._initial_y_home_btn = self._make_home_btn()
        initial_form.addRow('y', self._row_with_home_btn(
            self._initial_y, self._initial_y_home_btn))
        self._initial_z = self._make_pose_spinbox(_HOME_FALLBACK[1],
                                                  lo=0.1, hi=0.9)
        self._initial_z_home_btn = self._make_home_btn()
        initial_form.addRow('z', self._row_with_home_btn(
            self._initial_z, self._initial_z_home_btn))
        initial_outer.addLayout(initial_form)
        # Move-to-initial button: send the arm to the currently typed
        # initial pose without starting a run, so the operator can
        # confirm the pose is reachable + safe before launching the test.
        self._move_initial_btn = QPushButton('Move to initial')
        initial_outer.addWidget(self._move_initial_btn)
        layout.addWidget(initial_box)

        # Goal input: single-pose for accuracy/repeatability, free-form
        # multi-pose list for workspace coverage. Both kept in the
        # layout simultaneously; the test combo toggles which is
        # visible (cleaner than a QStackedWidget for two simple forms).
        self._goal_box = QGroupBox('Goal pose (workspace, metres)')
        goal_form = QFormLayout(self._goal_box)
        self._goal_y = self._make_pose_spinbox(_HOME_FALLBACK[0],
                                               lo=-0.4, hi=0.4)
        goal_form.addRow('y', self._goal_y)
        self._goal_z = self._make_pose_spinbox(_HOME_FALLBACK[1],
                                               lo=0.1, hi=0.9)
        goal_form.addRow('z', self._goal_z)
        layout.addWidget(self._goal_box)

        self._goals_box = QGroupBox(
            'Goals list (workspace, metres) - one "y, z" per line')
        goals_outer = QVBoxLayout(self._goals_box)
        self._goals_edit = QPlainTextEdit()
        self._goals_edit.setPlaceholderText(
            '0.0, 0.5\n0.1, 0.5\n-0.1, 0.5\n0.0, 0.6')
        self._goals_edit.setMaximumBlockCount(200)
        goals_outer.addWidget(self._goals_edit)
        layout.addWidget(self._goals_box)
        # Default invisible; toggled by _on_test_changed once we know
        # which test the user picked.
        self._goals_box.setVisible(False)

        btn_row = QHBoxLayout()
        self._start_btn = QPushButton('Start Run')
        self._continue_btn = QPushButton('Continue')
        self._continue_btn.setEnabled(False)
        # Reset = abort current run (stop in place) then drive arm back
        # to the typed initial pose. Distinct from Cancel which only
        # stops; useful when an iteration is going somewhere unexpected
        # and the operator wants the arm safely re-parked.
        self._reset_btn = QPushButton('Reset')
        # Cancel = emergency stop. Arm halts wherever it is.
        self._cancel_btn = QPushButton('Cancel')
        btn_row.addWidget(self._start_btn)
        btn_row.addWidget(self._continue_btn)
        btn_row.addWidget(self._reset_btn)
        btn_row.addWidget(self._cancel_btn)
        layout.addLayout(btn_row)

        self._detection_label = QLabel('detection: idle')
        layout.addWidget(self._detection_label)

        self._status_label = QLabel('idle')
        self._status_label.setStyleSheet('font-weight: bold;')
        layout.addWidget(self._status_label)

        self._progress = QProgressBar()
        self._progress.setRange(0, 1)
        self._progress.setValue(0)
        layout.addWidget(self._progress)

        # QTextEdit (rich text) instead of QPlainTextEdit so each line
        # can be coloured by severity. Maximum block count keeps the
        # widget bounded; the ring buffer behaviour is identical.
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.document().setMaximumBlockCount(1000)
        layout.addWidget(self._log)

        # Post-run banner: shown after every finalize so the operator
        # can triage the result (Keep / Delete / Open notebook) without
        # leaving the GUI. Hidden during a run.
        self._banner = self._build_banner()
        layout.addWidget(self._banner)
        self._banner.setVisible(False)

        # Calibration-only mode: hide the test-runner widgets so the
        # operator only sees the "Camera localization" group. The
        # dashboard launch sets the `camera_calibration_only` parameter
        # to true when the bringup was started with calibration:=true.
        if self._camera_calibration_only:
            self.setWindowTitle(
                'Volcaniarm Camera Calibration')
            for widget in (
                cfg_box, self._home_box, initial_box,
                self._goal_box, self._goals_box,
            ):
                widget.setVisible(False)
            # The Run / Continue / Reset / Cancel button row applies to
            # the test runner; in calibration-only mode only Cancel is
            # meaningful (it also cancels the camera calibration).
            self._start_btn.setVisible(False)
            self._continue_btn.setVisible(False)
            self._reset_btn.setVisible(False)
            self._detection_label.setVisible(False)
            self._progress.setVisible(False)

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
        btn.setToolTip('Reset this axis to the home FK value (theta=0,0)')
        btn.setMaximumWidth(60)
        return btn

    def _row_with_home_btn(self, spinbox: QDoubleSpinBox,
                           home_btn: QPushButton) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(spinbox)
        row.addWidget(home_btn)
        return row

    def _wire_signals(self):
        self._start_btn.clicked.connect(self._on_start_clicked)
        self._continue_btn.clicked.connect(self._on_continue_clicked)
        self._reset_btn.clicked.connect(self._on_reset_clicked)
        self._cancel_btn.clicked.connect(self._on_cancel_clicked)
        self._move_initial_btn.clicked.connect(self._on_move_initial_clicked)
        self._calibrate_btn.clicked.connect(self._on_calibrate_clicked)
        # Refresh the alignment status label periodically so it picks
        # up new result.yaml files written between dashboard sessions.
        self._align_timer = QTimer(self)
        self._align_timer.setInterval(2000)
        self._align_timer.timeout.connect(self._refresh_alignment_state)
        self._align_timer.start()
        self._refresh_alignment_state()
        self._initial_y_home_btn.clicked.connect(
            lambda: self._initial_y.setValue(self._home_fk_y))
        self._initial_z_home_btn.clicked.connect(
            lambda: self._initial_z.setValue(self._home_fk_z))
        self._test_combo.currentTextChanged.connect(self._on_test_changed)
        # Make sure the initial test selection has the right input
        # group visible (defaults to whichever the combo lands on after
        # restore_settings).
        self._on_test_changed(self._test_combo.currentText())
        self._bridge.status.connect(self._on_status)
        self._bridge.progress.connect(self._on_progress)
        self._bridge.finished.connect(self._on_finished)
        self._bridge.awaiting_continue.connect(self._on_awaiting_continue)
        self._bridge.detection_state.connect(self._on_detection_state)
        self._bridge.home_fk_resolved.connect(self._on_home_fk_resolved)
        self._bridge.camera_calib_finished.connect(
            self._on_camera_calib_finished)

    @Slot(str)
    def _on_test_changed(self, name: str):
        is_workspace = (name == 'workspace_coverage')
        is_repeatability = (name == 'repeatability')
        self._goals_box.setVisible(is_workspace)
        self._goal_box.setVisible(not is_workspace)
        # Home-confirm controls only apply to tests that gate on tag-
        # confirmed home (repeatability). Hide them otherwise so the
        # operator isn't presented with knobs that have no effect.
        self._home_box.setVisible(is_repeatability)
        # Workspace coverage visits each goal exactly once -- the
        # iterations spinbox doesn't apply. Disable it and pin to 1
        # so the operator isn't misled by a value the runner ignores.
        self._iterations.setEnabled(not is_workspace)
        if is_workspace:
            self._iterations.setValue(1)
        # Switching test type implies the operator is starting fresh:
        # cancel anything still running and put the dashboard back
        # into "ready to start" state. cancel() is safe when nothing
        # is running -- it just sets the events.
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
        and test-switch paths."""
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
        # Only overwrite spinboxes if the user (or restored settings)
        # hasn't already changed them away from the fallback. Without
        # this a late FK reply would clobber a value the operator just
        # typed.
        for sb, fallback_v in (
            (self._initial_y, _HOME_FALLBACK[0]),
            (self._goal_y, _HOME_FALLBACK[0]),
        ):
            if abs(sb.value() - fallback_v) < 1e-9:
                sb.setValue(y)
        for sb, fallback_v in (
            (self._initial_z, _HOME_FALLBACK[1]),
            (self._goal_z, _HOME_FALLBACK[1]),
        ):
            if abs(sb.value() - fallback_v) < 1e-9:
                sb.setValue(z)

    # -- button slots --------------------------------------------

    @Slot()
    def _on_start_clicked(self):
        test_name = self._test_combo.currentText()
        cls = TEST_REGISTRY[test_name]
        if test_name == 'workspace_coverage':
            try:
                goals = self._parse_goals_text(self._goals_edit.toPlainText())
            except ValueError as exc:
                self._log_msg(f'goals parse error: {exc}')
                self._status_label.setText(f'cannot start: {exc}')
                return
            if not goals:
                self._log_msg('goals list is empty; nothing to run')
                self._status_label.setText('cannot start: empty goals list')
                return
        else:
            goals = [(self._goal_y.value(), self._goal_z.value())]
        # Test classes still take a `targets` list (kept for backward
        # compat with iter_visits and any existing test subclasses);
        # the new runner reads `request.goals` directly so the targets
        # field is mostly bookkeeping now.
        try:
            test = cls(
                targets=goals,
                num_cycles=self._iterations.value(),
                settle_time=self._settle_time.value(),
                return_home_between_targets=True,
            )
        except ValueError as exc:
            self._log_msg(f'cannot start: {exc}')
            self._status_label.setText(f'cannot start: {exc}')
            return
        request = RunRequest(
            test=test,
            output_root=Path(DEFAULT_OUTPUT_DIR).expanduser(),
            initial_pose=(self._initial_y.value(), self._initial_z.value()),
            goals=tuple(goals),
            detection_max_age_s=self._fresh_window.value(),
            home_tol_m=self._home_tol_mm.value() / 1000.0,
            home_hold_frames=self._home_hold_frames.value(),
            home_timeout_s=self._home_timeout_s.value(),
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
        self._runner.reset_to(self._initial_y.value(), self._initial_z.value())
        self._reset_ui_state(status='resetting: returning arm to initial')

    @Slot()
    def _on_move_initial_clicked(self):
        self._runner.goto(self._initial_y.value(), self._initial_z.value())

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
        self._last_test_name = self._test_combo.currentText()
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

        Tries the installed share dir first (production install), then
        falls back to the source tree (developer running symlink-install
        without re-installing notebooks). Returns None if neither
        exists, so the Open button can disable itself.
        """
        if not self._last_test_name:
            return None
        candidates: list = []
        try:
            share = Path(get_package_share_directory('volcaniarm_calibration'))
            candidates.append(share / 'notebooks' / f'{self._last_test_name}.ipynb')
        except Exception:
            pass
        # Source-tree fallback. The widget lives at
        # <pkg>/volcaniarm_calibration/rqt/calibration_dashboard_widget.py;
        # the notebooks dir is two levels up under the package root.
        src_nb = (Path(__file__).resolve().parents[2]
                  / 'notebooks' / f'{self._last_test_name}.ipynb')
        candidates.append(src_nb)
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
        plugin_settings.set_value('test_type', self._test_combo.currentText())
        plugin_settings.set_value('initial_y', self._initial_y.value())
        plugin_settings.set_value('initial_z', self._initial_z.value())
        plugin_settings.set_value('goal_y', self._goal_y.value())
        plugin_settings.set_value('goal_z', self._goal_z.value())
        plugin_settings.set_value('iterations', self._iterations.value())
        plugin_settings.set_value('settle_time', self._settle_time.value())
        plugin_settings.set_value('fresh_window', self._fresh_window.value())
        plugin_settings.set_value('auto_continue', self._auto_continue.isChecked())
        plugin_settings.set_value('auto_hold', self._auto_hold.value())
        plugin_settings.set_value('home_tol_mm', self._home_tol_mm.value())
        plugin_settings.set_value('home_hold_frames', self._home_hold_frames.value())
        plugin_settings.set_value('home_timeout_s', self._home_timeout_s.value())
        plugin_settings.set_value('goals_text', self._goals_edit.toPlainText())

    def restore_settings(self, plugin_settings):
        v = plugin_settings.value('test_type')
        if v is not None:
            idx = self._test_combo.findText(v)
            if idx >= 0:
                self._test_combo.setCurrentIndex(idx)
        for key, widget in (
            ('initial_y', self._initial_y),
            ('initial_z', self._initial_z),
            ('goal_y', self._goal_y),
            ('goal_z', self._goal_z),
            ('iterations', self._iterations),
            ('settle_time', self._settle_time),
            ('fresh_window', self._fresh_window),
            ('auto_hold', self._auto_hold),
            ('home_tol_mm', self._home_tol_mm),
            ('home_hold_frames', self._home_hold_frames),
            ('home_timeout_s', self._home_timeout_s),
        ):
            v = plugin_settings.value(key)
            if v is not None:
                try:
                    widget.setValue(type(widget.value())(v))
                except (TypeError, ValueError):
                    pass
        v = plugin_settings.value('auto_continue')
        if v is not None:
            self._auto_continue.setChecked(str(v).lower() in ('1', 'true'))
        gt = plugin_settings.value('goals_text')
        if gt is not None:
            self._goals_edit.setPlainText(str(gt))
