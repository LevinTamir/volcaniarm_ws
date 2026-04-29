"""Qt widget for the calibration dashboard.

Human-in-the-loop flow for real-hardware calibration:

  1. Operator picks the test type, fills in initial and goal pose
     (workspace y, z metres), iterations, samples per visit.
  2. Click Start Run. The arm moves to the initial pose, captures a
     baseline reading of the apriltag base->ee transform, then begins
     the iteration loop.
  3. At each iteration the arm settles at the goal pose and the runner
     blocks. The operator physically repositions the real camera until
     both apriltags are visible. Once the runner sees a fresh
     ``apriltag_marker_base -> apriltag_marker_ee`` TF the Continue
     button enables. Click Continue to capture and move on; Cancel
     aborts the whole run.

UI is built programmatically (no .ui file) for simplicity.
"""

from __future__ import annotations

from pathlib import Path
import threading

from python_qt_binding.QtCore import Signal, Slot, QObject
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QComboBox, QDoubleSpinBox, QSpinBox, QPushButton, QLabel,
    QPlainTextEdit, QProgressBar,
)

from ..runner import (
    CalibrationRunner, RunRequest, TEST_REGISTRY,
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


class CalibrationDashboardWidget(QWidget):

    def __init__(self, node):
        super().__init__()
        self.setObjectName('CalibrationDashboardWidget')
        self.setWindowTitle('Volcaniarm Calibration Dashboard')

        self._node = node
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

        self._build_ui()
        self._wire_signals()
        self._defaults_from_fk_in_background()

    # -- UI assembly ----------------------------------------------

    def _build_ui(self):
        layout = QVBoxLayout(self)

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
        self._samples = QSpinBox()
        self._samples.setRange(1, 200)
        self._samples.setValue(20)
        cfg_form.addRow('samples per visit', self._samples)
        layout.addWidget(cfg_box)

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

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(1000)
        layout.addWidget(self._log)

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

    @Slot(str)
    def _on_test_changed(self, name: str):
        is_workspace = (name == 'workspace_coverage')
        self._goals_box.setVisible(is_workspace)
        self._goal_box.setVisible(not is_workspace)

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
                self._log.appendPlainText(f'goals parse error: {exc}')
                self._status_label.setText(f'cannot start: {exc}')
                return
            if not goals:
                self._log.appendPlainText('goals list is empty; nothing to run')
                self._status_label.setText('cannot start: empty goals list')
                return
        else:
            goals = [(self._goal_y.value(), self._goal_z.value())]
        # Test classes still take a `targets` list (kept for backward
        # compat with iter_visits and any existing test subclasses);
        # the new runner reads `request.goals` directly so the targets
        # field is mostly bookkeeping now.
        test = cls(
            targets=goals,
            num_cycles=self._iterations.value(),
            samples_per_visit=self._samples.value(),
            settle_time=2.0,
            sample_interval=0.1,
            return_home_between_targets=True,
        )
        request = RunRequest(
            test=test,
            output_root=Path(DEFAULT_OUTPUT_DIR).expanduser(),
            initial_pose=(self._initial_y.value(), self._initial_z.value()),
            goals=tuple(goals),
        )
        if self._runner.request_run(request):
            self._start_btn.setEnabled(False)
            self._continue_btn.setEnabled(False)
            self._progress.setRange(0, test.num_cycles * len(goals))
            self._progress.setValue(0)
            self._detection_label.setText('detection: idle')
            self._log.appendPlainText(
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
        self._runner.proceed()
        self._continue_btn.setEnabled(False)

    @Slot()
    def _on_cancel_clicked(self):
        # Stop in place; do not return the arm anywhere. Reset is the
        # button to use when you want the arm parked at the typed
        # initial pose after halting.
        self._runner.cancel()
        self._continue_btn.setEnabled(False)

    @Slot()
    def _on_reset_clicked(self):
        self._runner.reset_to(self._initial_y.value(), self._initial_z.value())
        self._continue_btn.setEnabled(False)

    @Slot()
    def _on_move_initial_clicked(self):
        self._runner.goto(self._initial_y.value(), self._initial_z.value())

    # -- runner-side slots ---------------------------------------

    @Slot(str)
    def _on_status(self, msg: str):
        self._status_label.setText(msg)
        self._log.appendPlainText(msg)

    @Slot(int, int)
    def _on_progress(self, current: int, total: int):
        self._progress.setRange(0, max(total, 1))
        self._progress.setValue(current)

    @Slot(int, int)
    def _on_awaiting_continue(self, iteration: int, total: int):
        self._status_label.setText(
            f'iteration {iteration}/{total}: position camera, then click Continue')

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

    @Slot(str, str)
    def _on_finished(self, run_dir: str, status: str):
        self._status_label.setText(f'run {status}')
        self._log.appendPlainText(f'output: {run_dir}')
        self._start_btn.setEnabled(True)
        self._continue_btn.setEnabled(False)
        self._detection_label.setText('detection: idle')
        self._detection_label.setStyleSheet('')

    def shutdown(self):
        self._runner.shutdown()

    def save_settings(self, plugin_settings):
        plugin_settings.set_value('test_type', self._test_combo.currentText())
        plugin_settings.set_value('initial_y', self._initial_y.value())
        plugin_settings.set_value('initial_z', self._initial_z.value())
        plugin_settings.set_value('goal_y', self._goal_y.value())
        plugin_settings.set_value('goal_z', self._goal_z.value())
        plugin_settings.set_value('iterations', self._iterations.value())
        plugin_settings.set_value('samples', self._samples.value())
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
            ('samples', self._samples),
        ):
            v = plugin_settings.value(key)
            if v is not None:
                try:
                    widget.setValue(type(widget.value())(v))
                except (TypeError, ValueError):
                    pass
        gt = plugin_settings.value('goals_text')
        if gt is not None:
            self._goals_edit.setPlainText(str(gt))
