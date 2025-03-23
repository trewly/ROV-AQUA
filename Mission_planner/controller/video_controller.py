import sys
import cv2
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal

class VideoReceiver(QWidget):
    frame_signal = pyqtSignal(np.ndarray)

    def __init__(self, udp_ip="169.254.54.121", udp_port=5002, save_path="received_video.mp4", parent=None):
        super().__init__(parent)
        self.setGeometry(0, 0, 960, 540)

        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 960, 540)

        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.save_path = save_path
        self.running = True

        self.video_writer = None

        self.video_thread = QThread()
        self.video_thread.run = self.run_video
        self.video_thread.start()

        self.frame_signal.connect(self.update_frame)

    def run_video(self):
        cap = None
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        fps = 30
        frame_size = (1920, 1080)

        while self.running:
            if cap is None:
                cap = cv2.VideoCapture(f"udp://{self.udp_ip}:{self.udp_port}", cv2.CAP_FFMPEG)
                if not cap.isOpened():
                    print("Đang chờ luồng video...")
                    cap.release()
                    cap = None
                    QtCore.QThread.msleep(500)
                    continue

            if self.video_writer is None:
                self.video_writer = cv2.VideoWriter(self.save_path, fourcc, fps, frame_size)

            ret, frame = cap.read()
            if ret:
                frame = cv2.flip(frame, -1)
                self.frame_signal.emit(frame)
                self.video_writer.write(frame)
            else:
                print("Mất kết nối video, thử kết nối lại...")
                cap.release()
                cap = None
                if self.video_writer:
                    self.video_writer.release()
                    self.video_writer = None
                QtCore.QThread.msleep(500)

        if cap:
            cap.release()
        if self.video_writer:
            self.video_writer.release()

    def update_frame(self, frame):
        frame_resized = cv2.resize(frame, (960, 540))
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        height, width, channel = frame_rgb.shape
        bytes_per_line = 3 * width
        qimg = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.running = False
        self.video_thread.quit()
        self.video_thread.wait()
        event.accept()
        print(f"Đã lưu video tại: {self.save_path}")
