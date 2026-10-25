"""Sensorless regression: SIGINT must not escape a Qt timer callback."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import tempfile
import unittest


class CalibrationShutdownTest(unittest.TestCase):
    def test_hidden_manager_sigint_supervises_child(self):
        scripts = Path(__file__).resolve().parents[1] / 'scripts'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / 'fake_camera'
            marker = root / 'child_stopped'
            ready = root / 'hidden_ready'
            binary.write_text(
                '#!/usr/bin/python3\n'
                'import signal, sys, time\n'
                'from pathlib import Path\n'
                'if sys.argv[1:] == ["--list-cameras"]:\n'
                ' print(\'CAMERA "Fake" "/dev/fake" 0\', flush=True)\n'
                ' sys.exit(0)\n'
                'stop = False\n'
                'def shutdown(*_):\n'
                ' global stop\n'
                ' stop = True\n'
                'signal.signal(signal.SIGINT, shutdown)\n'
                'while not stop: time.sleep(.01)\n'
                f'Path({str(marker)!r}).write_text("stopped")\n')
            binary.chmod(0o755)
            runner = root / 'runner.py'
            runner.write_text(
                'import sys, signal\n'
                f'sys.path.insert(0, {str(scripts)!r})\n'
                'import rclpy\n'
                'from rclpy.signals import SignalHandlerOptions\n'
                'from PyQt5.QtCore import QTimer, QProcess\n'
                'from PyQt5.QtWidgets import QApplication\n'
                'from thermal_camera_manager import CameraManager\n'
                'from pathlib import Path\n'
                'rclpy.init(signal_handler_options=SignalHandlerOptions.NO)\n'
                'app=QApplication([])\n'
                'app.setQuitOnLastWindowClosed(False)\n'
                'node=rclpy.create_node("hidden_shutdown_test")\n'
                f'window=CameraManager(node, {str(binary)!r})\n'
                'signal.signal(signal.SIGINT, lambda *_: window.close())\n'
                'def connect():\n'
                ' if window.devices.count():\n'
                '  connector.stop()\n'
                '  window.connect_camera()\n'
                'def hide():\n'
                ' if window.camera_process.state() == QProcess.Running:\n'
                '  hider.stop()\n'
                '  window.hide_controls.setChecked(True)\n'
                '  window.measurement_started()\n'
                f'  Path({str(ready)!r}).write_text("ready")\n'
                'connector=QTimer(); connector.timeout.connect(connect); connector.start(20)\n'
                'hider=QTimer(); hider.timeout.connect(hide); hider.start(20)\n'
                'window.show()\n'
                'app.exec_()\n'
                'window.log_file.close()\n'
                'node.destroy_node(); rclpy.shutdown()\n')
            process = subprocess.Popen([sys.executable, str(runner)],
                env=dict(os.environ, QT_QPA_PLATFORM='offscreen', ROS_DOMAIN_ID='194'),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(ready.exists(), 'Hidden manager did not become ready')
                time.sleep(.15)  # let fake child install its signal handler
                self.assertIsNone(process.poll())
                self.assertFalse(marker.exists())
                process.send_signal(signal.SIGINT)
                output, _ = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, output)
                self.assertTrue(marker.exists(), output)
                self.assertNotIn('Traceback', output)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_single_and_repeated_sigint(self):
        script = Path(__file__).resolve().parents[1] / 'scripts' / 'thermal_calibration_gui.py'
        environment = dict(os.environ, QT_QPA_PLATFORM='offscreen', ROS_DOMAIN_ID='192')
        for repeat in (1, 2):
            with self.subTest(repeat=repeat):
                process = subprocess.Popen(
                    [sys.executable, str(script)], env=environment,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                try:
                    time.sleep(1.5)
                    for _ in range(repeat):
                        if process.poll() is None:
                            process.send_signal(signal.SIGINT)
                    output, _ = process.communicate(timeout=5)
                    self.assertEqual(process.returncode, 0, output)
                    self.assertNotIn('Traceback', output)
                    self.assertNotIn('KeyboardInterrupt', output)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()


if __name__ == '__main__':
    unittest.main()
