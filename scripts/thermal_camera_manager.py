#!/usr/bin/env python3
"""Device selection and supervised calibration; the C++ child owns the SDK."""
import os
import shlex
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_prefix
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import QApplication, QCheckBox, QComboBox, QHBoxLayout, QPushButton, QWidget
from sensor_msgs.msg import Image

from thermal_calibration_gui import CalibrationWindow


class CameraManager(CalibrationWindow):
    def __init__(self, node, binary):
        super().__init__(node, close_on_confirm=False)
        self.setWindowTitle('열화상 카메라 검색 · 연결 · 제어')
        self.hidden_supervisor = False
        self.binary = binary
        self.exiting = False
        self.stopping = False
        self.last_state = 0
        self.scan_output = ''
        log_dir = Path(os.environ.get('ROS_LOG_DIR', str(Path.home() / '.ros/log')))
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / ('thermal_manager_' + datetime.now().strftime('%Y%m%d_%H%M%S') +
                              f'_{os.getpid()}.log')
        self.log_file = log_path.open('a', buffering=1)
        self.log(f'[MANAGER] log={log_path}\n')
        row = QWidget(self)
        controls = QHBoxLayout(row)
        self.devices = QComboBox()
        self.scan_button = QPushButton('장치 검색')
        self.connect_button = QPushButton('연결')
        self.disconnect_button = QPushButton('연결 해제')
        for widget in (self.devices, self.scan_button, self.connect_button,
                       self.disconnect_button):
            controls.addWidget(widget)
        self.layout().insertWidget(0, row)
        self.disable_auto_ffc = QCheckBox('자동 캘리브레이션 끄기')
        self.show_preview = QCheckBox('영상 미리보기 표시')
        self.show_preview.setChecked(True)
        self.hide_controls = QCheckBox('측정 시작 후 컨트롤 창 숨기기')
        for option in (self.disable_auto_ffc, self.show_preview, self.hide_controls):
            self.layout().insertWidget(1, option)
        self.disable_auto_ffc.toggled.connect(self.run.setVisible)
        self.show_preview.toggled.connect(self.toggle_preview)
        self.run.setVisible(False)
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(False)
        self.camera_process = QProcess(self)
        self.camera_process.setProcessChannelMode(QProcess.MergedChannels)
        self.camera_process.readyReadStandardOutput.connect(self.driver_output)
        self.camera_process.finished.connect(self.driver_finished)
        self.camera_process.errorOccurred.connect(self.driver_error)
        self.camera_process.started.connect(self.driver_started)
        self.scanner = QProcess(self)
        self.scanner.setProcessChannelMode(QProcess.MergedChannels)
        self.scanner.readyReadStandardOutput.connect(self.scan_read)
        self.scanner.finished.connect(self.scan_finished)
        self.scanner.errorOccurred.connect(self.scan_error)
        self.scan_button.clicked.connect(self.scan)
        self.connect_button.clicked.connect(self.connect_camera)
        self.disconnect_button.clicked.connect(self.disconnect_camera)
        # Continue displaying frames after manual confirmation enables measurements.
        self.live_image = node.create_subscription(Image, '/thermal/image_color', self.preview, 1)
        self.supervisor = QTimer(self)
        self.supervisor.timeout.connect(self.supervise)
        self.supervisor.start(100)
        QTimer.singleShot(0, self.scan)

    def toggle_preview(self, enabled):
        self.preview_enabled = enabled
        self.image.setVisible(enabled)
        if not enabled:
            self.image.clear()

    def measurement_started(self):
        self.confirm.setEnabled(False)
        if self.hide_controls.isChecked():
            self.hidden_supervisor = True
            self.preview_enabled = False
            self.timer.stop()
            for subscription in (self.sub_image, self.sub_state, self.live_image):
                self.node.destroy_subscription(subscription)
            self.hide()
            self.log('[MANAGER] Controls hidden; driver continues. Ctrl+C stops acquisition.\n')

    def scan(self):
        if self.camera_process.state() != QProcess.NotRunning:
            return
        self.devices.clear()
        self.scan_output = ''
        self.scan_button.setEnabled(False)
        self.connect_button.setEnabled(False)
        self.status.setText('장치 검색 중 (연결·센서 설정 변경 없음)')
        self.scanner.start(self.binary, ['--list-cameras'])

    def scan_read(self):
        self.scan_output += bytes(self.scanner.readAllStandardOutput()).decode(errors='replace')

    def scan_finished(self, code, exit_status):
        self.scan_read()
        for line in self.scan_output.splitlines():
            if not line.startswith('CAMERA '):
                continue
            try:
                _, name, port, index = shlex.split(line)
            except ValueError:
                continue
            self.devices.addItem(f'{name} | {port} | video index {index}', port)
        self.scan_button.setEnabled(not self.exiting)
        self.connect_button.setEnabled(self.devices.count() > 0 and not self.exiting)
        self.status.setText(f'{self.devices.count()}개 장치 발견' if code == 0
                            else '검색 실패: ' + self.scan_output[-300:])

    def scan_error(self, error):
        if error == QProcess.FailedToStart:
            self.scan_button.setEnabled(True)
            self.status.setText('검색 실행 실패: ' + self.scanner.errorString())

    def connect_camera(self):
        port = self.devices.currentData()
        if not port or self.camera_process.state() != QProcess.NotRunning or self.exiting:
            return
        self.stopping = False
        self.last_image = self.last_state = 0
        self.ready = False
        self.image.setText('연결 중 · 영상 대기')
        self.status.setText('연결 중')
        self.scan_button.setEnabled(False)
        self.devices.setEnabled(False)
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)
        self.disable_auto_ffc.setEnabled(False)
        # Only explicit opt-in changes the SDK FFC mode.
        self.camera_process.start(self.binary, [
            '--ros-args', '-r', '__node:=thermal_node',
            '-p', 'device_port:=' + port,
            '-p', 'gain_mode:=unchanged', '-p', 'startup_calibration:=true',
            '-p', 'disable_auto_ffc:=' + str(self.disable_auto_ffc.isChecked()).lower(),
            '-p', 'ffc_interval_s:=0.0', '-p', 'poll_rate_hz:=9.0'])

    def driver_output(self):
        output = bytes(self.camera_process.readAllStandardOutput()).decode(errors='replace')
        self.log(output)

    def log(self, output):
        print(output, end='', flush=True)
        self.log_file.write(output)

    def driver_error(self, error):
        if error == QProcess.FailedToStart:
            self.driver_finished(-1, QProcess.CrashExit)

    def driver_started(self):
        if self.exiting or self.stopping:
            self.stopping = False
            self.disconnect_camera()

    def disconnect_camera(self):
        if self.camera_process.state() != QProcess.NotRunning and not self.stopping:
            self.stopping = True
            self.stop_started = time.monotonic()
            self.disconnect_button.setEnabled(False)
            self.status.setText('수집 종료 → SDK Close 완료 대기')
            # No terminate/kill escalation here: keep SDK teardown uninterrupted.
            pid = int(self.camera_process.processId())
            if pid > 0:
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        self.run.setEnabled(False)
        self.confirm.setEnabled(False)

    def driver_finished(self, code, exit_status):
        self.driver_output()
        self.log(f'[MANAGER] child exit={code}, Qt status={exit_status}\n')
        self.stopping = False
        if self.hidden_supervisor:
            self.exiting = True
        self.ready = self.pending = False
        self.last_image = self.last_state = 0
        self.run.setEnabled(False)
        self.confirm.setEnabled(False)
        self.disable_auto_ffc.setEnabled(True)
        self.devices.setEnabled(True)
        self.scan_button.setEnabled(not self.exiting)
        self.connect_button.setEnabled(self.devices.count() > 0 and not self.exiting)
        self.disconnect_button.setEnabled(False)
        self.image.setText('카메라 연결 해제')
        self.status.setText(f'드라이버 종료 (code={code}) · 터미널의 [CLOSE] complete 확인')

    def state(self, msg):
        if not self.stopping and self.camera_process.state() == QProcess.Running:
            super().state(msg)

    def preview(self, msg):
        if not self.stopping and self.camera_process.state() == QProcess.Running:
            super().preview(msg)

    def request(self, client, finish):
        if (self.stopping or self.camera_process.state() != QProcess.Running or
                time.monotonic() - self.last_image > 0.5):
            return
        super().request(client, finish)

    def supervise(self):
        if self.stopping:
            self.run.setEnabled(False)
            self.confirm.setEnabled(False)
            if time.monotonic() - self.stop_started > 15:
                self.status.setText('SDK 종료 지연 · 강제 종료 없이 기다리는 중 (터미널 로그 확인)')
        if (self.exiting and self.camera_process.state() == QProcess.NotRunning and
                self.scanner.state() == QProcess.NotRunning):
            self.timer.stop()
            QApplication.quit()

    def closeEvent(self, event):
        self.exiting = True
        self.scan_button.setEnabled(False)
        self.connect_button.setEnabled(False)
        self.disconnect_camera()
        if self.scanner.state() != QProcess.NotRunning:
            self.scanner.terminate()  # Scanner never opens the camera.
        event.ignore()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('thermal_camera_manager')
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)
    binary = os.path.join(get_package_prefix('thermal_cam_driver'),
                          'lib', 'thermal_cam_driver', 'thermal_node')
    window = CameraManager(node, binary)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: window.close())
    window.show()
    try:
        app.exec_()
    finally:
        window.log_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
