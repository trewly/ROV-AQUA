import sys
import cv2
import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QPushButton
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt

class VideoReceiver(QWidget):
    """Class nhận video qua UDP và phát trên QLabel"""
    frame_signal = pyqtSignal(np.ndarray)

    def __init__(self, udp_ip="169.254.54.121", udp_port=5000, parent=None):
        super().__init__(parent)
        self.setGeometry(0, 0, 960, 540)

        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 960, 540)

        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.running = True
        self.video_thread = QThread()
        self.video_thread.run = self.run_video
        self.video_thread.start()

        self.frame_signal.connect(self.update_frame)

    def run_video(self):
        """Chờ có dữ liệu video trước khi hiển thị"""
        cap = None
        while self.running:
            if cap is None:
                cap = cv2.VideoCapture(f"udp://{self.udp_ip}:{self.udp_port}", cv2.CAP_FFMPEG)
                if not cap.isOpened():
                    print("Đang chờ luồng video...")
                    cap.release()
                    cap = None
                    QtCore.QThread.msleep(500)  # Chờ 500ms trước khi thử lại
                    continue
            
            ret, frame = cap.read()
            if ret:
                self.frame_signal.emit(frame)
            else:
                print("Mất kết nối video, thử kết nối lại...")
                cap.release()
                cap = None
                QtCore.QThread.msleep(500)  # Chờ trước khi thử lại

        if cap:
            cap.release()

    def update_frame(self, frame):
        """Hiển thị khung hình trên QLabel"""
        frame = cv2.resize(frame, (960, 540))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimg = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        """Đóng ứng dụng và dừng video thread"""
        self.running = False
        self.video_thread.quit()
        self.video_thread.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoReceiver()
    window.show()
    sys.exit(app.exec_())