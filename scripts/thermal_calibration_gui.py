#!/usr/bin/env python3
"""Optional manual calibration controls; no SDK access from Qt."""
import json
import signal
import sys
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QMessageBox


class CalibrationWindow(QWidget):
    def __init__(self, node, close_on_confirm=True):
        super().__init__()
        self.node = node
        self.close_on_confirm = close_on_confirm
        self.preview_enabled = True
        self.last_image = 0
        self.setWindowTitle('열화상 카메라 제어')
        self.resize(700, 600)
        layout = QVBoxLayout(self)
        self.instructions = QLabel('수동 보정은 자동 캘리브레이션 끄기를 선택한 경우에만 사용할 수 있습니다.\n카메라 제조사의 보정 절차를 따르십시오.')
        layout.addWidget(self.instructions)
        self.image = QLabel('카메라 연결 대기')
        layout.addWidget(self.image)
        self.status = QLabel('상태 수신 대기')
        layout.addWidget(self.status)
        self.run = QPushButton('보정 실행 / 다시 보정')
        self.run.setEnabled(False)
        self.confirm = QPushButton('영상 확인 완료 · 측정 시작')
        self.confirm.setEnabled(False)
        layout.addWidget(self.run)
        layout.addWidget(self.confirm)
        self.run_client = node.create_client(Trigger, '/thermal/calibration/run')
        self.confirm_client = node.create_client(Trigger, '/thermal/calibration/confirm')
        self.run.clicked.connect(self.calibrate)
        self.confirm.clicked.connect(lambda: self.request(self.confirm_client, True))
        self.sub_image = node.create_subscription(Image, '/thermal/calibration/preview', self.preview, 1)
        self.sub_state = node.create_subscription(String, '/thermal/ffc/status', self.state, 10)
        self.last_state = 0
        self.ready = False
        self.pending = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)

    def preview(self, msg):
        if msg.encoding != 'bgr8':
            return
        self.last_image = time.monotonic()
        if not self.preview_enabled:
            return
        image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_RGB888).rgbSwapped().copy()
        self.image.setPixmap(QPixmap.fromImage(image).scaledToWidth(640))

    def state(self, msg):
        try:
            state = json.loads(msg.data)['state']
        except (ValueError, KeyError):
            return
        self.last_state = time.monotonic()
        self.ready = state == 'awaiting_confirmation'
        self.status.setText(state + (' — 영상 확인 후 측정 시작' if self.ready else ''))

    def calibrate(self):
        if QMessageBox.question(self, '수동 보정', '카메라 제조사의 보정 준비 절차를 완료했습니까?') == QMessageBox.Yes:
            self.request(self.run_client, False)

    def request(self, client, finish):
        if not client.service_is_ready():
            self.status.setText('드라이버 서비스 연결 대기')
            return
        self.pending = True
        self.confirm.setEnabled(False)
        self.run.setEnabled(False)
        future = client.call_async(Trigger.Request())
        def done(result):
            self.pending = False
            try:
                response = result.result()
                self.status.setText(response.message)
                if response.success and finish:
                    self.measurement_started()
            except Exception as error:
                self.status.setText(str(error))
        future.add_done_callback(done)

    def measurement_started(self):
        if self.close_on_confirm:
            self.close()

    def tick(self):
        if not rclpy.ok(context=self.node.context):
            self.timer.stop()
            QApplication.quit()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except ExternalShutdownException:
            self.timer.stop()
            QApplication.quit()
        except RCLError:
            if rclpy.ok(context=self.node.context):
                raise
            self.timer.stop()
            QApplication.quit()
        self.run.setEnabled(not self.pending and
                            time.monotonic() - self.last_image < 0.5 and
                            time.monotonic() - self.last_state < 0.5)
        self.confirm.setEnabled(self.ready and not self.pending and
                                time.monotonic() - self.last_state < 0.5 and
                                time.monotonic() - self.last_image < 0.5)

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main():
    rclpy.init()
    node = rclpy.create_node('thermal_calibration_gui')
    app = QApplication(sys.argv)
    window = CalibrationWindow(node)
    # Python's default SIGINT handler can raise inside a Qt timer callback,
    # outside the app.exec_() try block. Request Qt shutdown instead, and leave
    # ROS context destruction to the single cleanup path below.
    def request_shutdown(signum, frame):
        window.timer.stop()
        app.quit()

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    window.show()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        window.timer.stop()
        node.destroy_node()
        if rclpy.ok(context=node.context):
            rclpy.shutdown(context=node.context)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
