"""Qt widget for the calibration dashboard.

UI is built programmatically (no .ui file) for simplicity in this first
pass. Wiring: user fills in the form, clicks Preview to publish RViz
markers and pause for approval, clicks Approve to release the move,
Cancel to abort the run.
"""

from __future__ import annotations

from pathlib import Path

from python_qt_binding.QtCore import Signal, Slot, Qt, QObject
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QComboBox, QDoubleSpinBox, QSpinBox, QPushButton, QLabel,
    QPlainTextEdit, QProgressBar,
)

from ..runner import (
    CalibrationRunner, RunRequest, PreviewPayload, TEST_REGISTRY,
)
from .preview_publisher import PreviewPublisher


DEFAULT_OUTPUT_DIR = '~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data'


class _RunnerBridge(QObject):
    """Marshals runner callbacks (called from worker thread) onto Qt signals."""
    status = Signal(str)
    preview = Signal(object)
    progress = Signal(int, int)
    finished = Signal(str, str)


class CalibrationDashboardWidget(QWidget):

    def __init__(self, node):
        super().__init__()
        self.setObjectName('CalibrationDashboardWidget')
        self.setWindowTitle('Volcaniarm Calibration Dashboard')

        self._node = node
        self._bridge = _RunnerBridge()
        self._runner = CalibrationRunner(node)
        self._runner.status_cb = self._bridge.status.emit
        self._runner.preview_cb = self._bridge.preview.emit
        self._runner.progress_cb = self._bridge.progress.emit
        self._runner.finished_cb = lambda p, s: self._bridge.finished.emit(str(p), s)
        self._preview_pub = PreviewPublisher(node)

        self._build_ui()
        self._wire_signals()

    # -- UI assembly ----------------------------------------------

    def _build_ui(self):
        layout = QVBoxLayout(self)

        cfg_box = QGroupBox('Test configuration')
        form = QFormLayout(cfg_box)
        self._test_combo = QComboBox()
        for name in TEST_REGISTRY.keys():
            self._test_combo.addItem(name)
        form.addRow('test type', self._test_combo)
        self._target_y = QDoubleSpinBox()
        self._target_y.setRange(-0.4, 0.4)
        self._target_y.setSingleStep(0.05)
        self._target_y.setDecimals(3)
        self._target_y.setValue(0.0)
        form.addRow('target y [m]', self._target_y)
        self._target_z = QDoubleSpinBox()
        self._target_z.setRange(0.1, 0.9)
        self._target_z.setSingleStep(0.05)
        self._target_z.setDecimals(3)
        self._target_z.setValue(0.5)
        form.addRow('target z [m]', self._target_z)
        self._iterations = QSpinBox()
        self._iterations.setRange(1, 100)
        self._iterations.setValue(3)
        form.addRow('iterations', self._iterations)
        self._samples = QSpinBox()
        self._samples.setRange(1, 200)
        self._samples.setValue(20)
        form.addRow('samples per visit', self._samples)
        layout.addWidget(cfg_box)

        btn_row = QHBoxLayout()
        self._preview_btn = QPushButton('Preview')
        self._approve_btn = QPushButton('Approve')
        self._approve_btn.setEnabled(False)
        self._cancel_btn = QPushButton('Cancel')
        btn_row.addWidget(self._preview_btn)
        btn_row.addWidget(self._approve_btn)
        btn_row.addWidget(self._cancel_btn)
        layout.addLayout(btn_row)

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

    def _wire_signals(self):
        self._preview_btn.clicked.connect(self._on_preview_clicked)
        self._approve_btn.clicked.connect(self._on_approve_clicked)
        self._cancel_btn.clicked.connect(self._on_cancel_clicked)
        self._bridge.status.connect(self._on_status)
        self._bridge.preview.connect(self._on_preview_payload)
        self._bridge.progress.connect(self._on_progress)
        self._bridge.finished.connect(self._on_finished)

    # -- button slots --------------------------------------------

    @Slot()
    def _on_preview_clicked(self):
        cls = TEST_REGISTRY[self._test_combo.currentText()]
        targets = [(self._target_y.value(), self._target_z.value())]
        test = cls(
            targets=targets,
            num_cycles=self._iterations.value(),
            samples_per_visit=self._samples.value(),
            settle_time=2.0,
            sample_interval=0.1,
            return_home_between_targets=True,
        )
        request = RunRequest(
            test=test,
            output_root=Path(DEFAULT_OUTPUT_DIR).expanduser(),
            auto_approve=False,
        )
        if self._runner.request_run(request):
            self._approve_btn.setEnabled(False)
            self._preview_btn.setEnabled(False)
            self._progress.setRange(0, test.total_visits())
            self._progress.setValue(0)
            self._log.appendPlainText(f'requested run: {test.name}')

    @Slot()
    def _on_approve_clicked(self):
        self._runner.approve()
        self._approve_btn.setEnabled(False)
        self._preview_pub.clear_preview()

    @Slot()
    def _on_cancel_clicked(self):
        self._runner.cancel()
        self._preview_pub.clear_preview()

    # -- runner-side slots ---------------------------------------

    @Slot(str)
    def _on_status(self, msg: str):
        self._status_label.setText(msg)
        self._log.appendPlainText(msg)

    @Slot(object)
    def _on_preview_payload(self, payload):
        if not isinstance(payload, PreviewPayload):
            return
        label = (f'cycle {payload.cycle}/{payload.cycle_total} '
                 f'target {payload.target_idx}/{payload.target_total}')
        path = payload.joint_path_xyz or [payload.fk_xyz]
        self._preview_pub.publish_preview(
            target_xyz=payload.fk_xyz,
            joint_path_xyz=path,
            label=label,
        )
        self._approve_btn.setEnabled(True)

    @Slot(int, int)
    def _on_progress(self, current: int, total: int):
        self._progress.setRange(0, max(total, 1))
        self._progress.setValue(current)

    @Slot(str, str)
    def _on_finished(self, run_dir: str, status: str):
        self._status_label.setText(f'run {status}')
        self._log.appendPlainText(f'output: {run_dir}')
        self._preview_btn.setEnabled(True)
        self._approve_btn.setEnabled(False)
        self._preview_pub.clear_preview()

    def shutdown(self):
        self._runner.shutdown()

    def save_settings(self, plugin_settings):
        plugin_settings.set_value('test_type', self._test_combo.currentText())
        plugin_settings.set_value('target_y', self._target_y.value())
        plugin_settings.set_value('target_z', self._target_z.value())
        plugin_settings.set_value('iterations', self._iterations.value())
        plugin_settings.set_value('samples', self._samples.value())

    def restore_settings(self, plugin_settings):
        v = plugin_settings.value('test_type')
        if v is not None:
            idx = self._test_combo.findText(v)
            if idx >= 0:
                self._test_combo.setCurrentIndex(idx)
        for key, widget in (
            ('target_y', self._target_y),
            ('target_z', self._target_z),
            ('iterations', self._iterations),
            ('samples', self._samples),
        ):
            v = plugin_settings.value(key)
            if v is not None:
                try:
                    widget.setValue(type(widget.value())(v))
                except (TypeError, ValueError):
                    pass
