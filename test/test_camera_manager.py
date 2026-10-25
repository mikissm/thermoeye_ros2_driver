"""No hardware: discovery, explicit selection and graceful child shutdown."""
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import rclpy
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication
from thermal_camera_manager import CameraManager


class ManagerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.app = QApplication.instance() or QApplication([])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def wait_for(self, predicate):
        until = time.monotonic() + 5
        while not predicate() and time.monotonic() < until:
            self.app.processEvents()
            time.sleep(0.01)
        self.assertTrue(predicate())

    def test_preview_and_hidden_supervision(self):
        from sensor_msgs.msg import Image
        node = rclpy.create_node('manager_options_test')
        window = CameraManager(node, '/nonexistent')
        try:
            self.assertFalse(window.disable_auto_ffc.isChecked())
            self.assertTrue(window.run.isHidden())
            window.disable_auto_ffc.setChecked(True)
            self.assertFalse(window.run.isHidden())
            window.show_preview.setChecked(False)
            msg = Image(encoding='bgr8', width=2, height=2, step=6, data=bytes(12))
            # Base callback tracks frame freshness without converting/displaying it.
            from thermal_calibration_gui import CalibrationWindow
            CalibrationWindow.preview(window, msg)
            self.assertGreater(window.last_image, 0)
            self.assertTrue(window.image.isHidden())
            self.assertIsNone(window.image.pixmap())
            window.hide_controls.setChecked(True)
            window.measurement_started()
            self.assertTrue(window.hidden_supervisor)
            self.assertFalse(window.timer.isActive())
            self.assertTrue(window.supervisor.isActive())
            self.assertFalse(window.exiting)
        finally:
            window.timer.stop()
            window.supervisor.stop()
            if window.scanner.state() != QProcess.NotRunning:
                window.scanner.kill()
                window.scanner.waitForFinished(2000)
            window.log_file.close()
            node.destroy_node()

    def test_selection_and_close_waits_for_child(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'fake_camera'
            marker = Path(directory) / 'arguments'
            binary.write_text(
                '#!/usr/bin/python3\n'
                'import signal, sys, time\n'
                'from pathlib import Path\n'
                'if sys.argv[1:] == ["--list-cameras"]:\n'
                ' print(\'CAMERA "TMC test" "/dev/ttyACM99" 6\', flush=True)\n'
                ' sys.exit(0)\n'
                'stop = False\n'
                'def shutdown(*_):\n'
                ' global stop\n'
                ' stop = True\n'
                'signal.signal(signal.SIGINT, shutdown)\n'
                f'Path({str(marker)!r}).write_text("\\n".join(sys.argv[1:]))\n'
                'while not stop: time.sleep(.01)\n'
                'time.sleep(.2)\n'
                'print("[CLOSE] complete elapsed_s=0.2", flush=True)\n')
            binary.chmod(0o755)
            node = rclpy.create_node('manager_test')
            window = CameraManager(node, str(binary))
            try:
                self.wait_for(lambda: window.devices.count() == 1)
                self.assertFalse(marker.exists(), 'Scanning must never connect')
                self.assertFalse(window.run.isEnabled())
                window.connect_camera()
                self.wait_for(marker.exists)
                arguments = marker.read_text()
                self.assertIn('device_port:=/dev/ttyACM99', arguments)
                self.assertIn('startup_calibration:=true', arguments)
                self.assertIn('disable_auto_ffc:=false', arguments)
                self.assertFalse(window.disable_auto_ffc.isEnabled())
                self.assertIn('gain_mode:=unchanged', arguments)
                self.assertIn('ffc_interval_s:=0.0', arguments)
                window.hide_controls.setChecked(True)
                window.measurement_started()
                self.assertEqual(window.camera_process.state(), QProcess.Running)
                self.assertFalse(window.isVisible())
                self.assertFalse(window.timer.isActive())
                window.close()
                self.assertTrue(window.stopping)
                self.assertFalse(window.connect_button.isEnabled())
                self.wait_for(lambda: window.camera_process.state() == QProcess.NotRunning)
                window.log_file.flush()
                self.assertIn('[CLOSE] complete', Path(window.log_file.name).read_text())
                self.assertFalse(window.run.isEnabled())
            finally:
                window.timer.stop()
                window.supervisor.stop()
                if window.camera_process.state() != QProcess.NotRunning:
                    window.camera_process.kill()
                    window.camera_process.waitForFinished(2000)
                window.log_file.close()
                node.destroy_node()


if __name__ == '__main__':
    unittest.main()
