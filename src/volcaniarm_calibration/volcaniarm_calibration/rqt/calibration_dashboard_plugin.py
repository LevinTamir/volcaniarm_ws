"""rqt plugin entrypoint for the Volcaniarm calibration dashboard."""

from qt_gui.plugin import Plugin

from .calibration_dashboard_widget import CalibrationDashboardWidget


class CalibrationDashboardPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('VolcaniarmCalibrationDashboard')
        node = getattr(context, 'node', None)
        if node is None:
            # Older rqt API: rqt_gui_py creates a default node.
            from rqt_gui_py.plugin import Plugin as PyPlugin  # noqa: F401
            raise RuntimeError(
                'rqt context did not provide a node. Run via rqt_gui '
                'with --force-discover so the shared rclpy node is set.')

        self._widget = CalibrationDashboardWidget(node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f'{self._widget.windowTitle()} ({context.serial_number()})')
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, global_settings, perspective_settings):
        self._widget.save_settings(perspective_settings)

    def restore_settings(self, global_settings, perspective_settings):
        self._widget.restore_settings(perspective_settings)
