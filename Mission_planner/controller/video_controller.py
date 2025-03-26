import sys
import cv2
import numpy as np
import time
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, Qt

class VideoWorker(QThread):
    frame_signal = pyqtSignal(np.ndarray)
    connection_signal = pyqtSignal(bool)
    
    def __init__(self, udp_ip, udp_port, save_path):
        super().__init__()
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.save_path = save_path
        self.running = True
        self.connected = False
        self.video_writer = None
        
    def run(self):
        cap = None
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        fps = 30
        frame_size = (1920, 1080)
        reconnect_delay = 0.5
        last_frame_time = 0
        frame_timeout = 2.0
        retry_count = 0
        max_retries = 20  # Số lần thử lại tối đa

        while self.running:
            current_time = time.time()
            
            if cap is None:
                try:
                    if (retry_count > 10):
                        time.sleep(2) 
                    
                    retry_count += 1
                    
                    gstream = f"udp://{self.udp_ip}:{self.udp_port}?overrun_nonfatal=1&fifo_size=50000000"
                    print(f"Attempt {retry_count}: Connecting to video stream {gstream}")
                    
                    cap = cv2.VideoCapture(gstream, cv2.CAP_FFMPEG)
                    
                    ret, _ = cap.read()
                    if not ret or not cap.isOpened():
                        if self.connected:
                            self.connected = False
                            self.connection_signal.emit(False)
                        cap.release()
                        cap = None
                        time.sleep(reconnect_delay)
                        continue
                    
                    retry_count = 0
                    print("Video stream connected successfully")
                    
                except Exception as e:
                    print(f"Connection error: {e}")
                    if cap:
                        cap.release()
                    cap = None
                    time.sleep(reconnect_delay)
                    continue

            if self.video_writer is None and cap is not None:
                try:
                    self.video_writer = cv2.VideoWriter(self.save_path, fourcc, fps, frame_size)
                except Exception as e:
                    print(f"Failed to create video writer: {e}")

            try:
                ret, frame = cap.read()
                if ret:
                    if not self.connected:
                        self.connected = True
                        self.connection_signal.emit(True)
                    
                    last_frame_time = current_time
                    
                    frame = cv2.flip(frame, -1) 
                    self.frame_signal.emit(frame.copy())
                    
                    if self.video_writer:
                        try:
                            self.video_writer.write(frame)
                        except Exception as e:
                            print(f"Error writing to video file: {e}")
                else:
                    if current_time - last_frame_time > frame_timeout:
                        if self.connected:
                            self.connected = False
                            self.connection_signal.emit(False)
                            
                        print("Video connection lost, reconnecting...")
                        cap.release()
                        cap = None
                        
                        if self.video_writer:
                            self.video_writer.release()
                            self.video_writer = None
                        
                        time.sleep(reconnect_delay)
            except Exception as e:
                print(f"Error processing video frame: {e}")
                if cap:
                    cap.release()
                cap = None
                time.sleep(reconnect_delay)

        if cap:
            cap.release()
        if self.video_writer:
            self.video_writer.release()

    def stop(self):
        self.running = False
        self.wait(3000)
        
        if not self.isFinished():
            self.terminate()

class VideoReceiver(QWidget):
    def __init__(self, udp_ip="169.254.54.121", udp_port=5000, save_path="received_video.mp4", parent=None):
        super().__init__(parent)
        self.setGeometry(0, 0, 960, 540)
        
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 960, 540)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setText("Waiting for video...")
        self.label.setStyleSheet("background-color: black; color: white; font-size: 20px;")
        
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.save_path = save_path
        
        self.worker = VideoWorker(udp_ip, udp_port, save_path)
        self.worker.frame_signal.connect(self.update_frame)
        self.worker.connection_signal.connect(self.update_connection_status)
        self.worker.start()
        
        self.frame_count = 0
        self.last_fps_update = time.time()
        self.fps = 0

    def update_frame(self, frame):
        self.frame_count += 1
        current_time = time.time()
        
        time_diff = current_time - self.last_fps_update
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_fps_update = current_time
        
        try:
            frame_resized = cv2.resize(frame, (960, 540))
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            
            cv2.putText(
                frame_rgb, 
                f"FPS: {self.fps:.1f}", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (0, 255, 0), 
                2
            )
            
            height, width, channel = frame_rgb.shape
            bytes_per_line = 3 * width
            qimg = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            self.label.setPixmap(pixmap)
        except Exception as e:
            print(f"Error updating frame: {e}")

    def update_connection_status(self, connected):
        if connected:
            print("Video stream connected")
        else:
            print("Video stream disconnected")
            self.label.setText("Connection lost. Reconnecting...")
            
    def closeEvent(self, event):
        print("Closing video receiver...")
        self.worker.stop()
        print(f"Video saved to: {self.save_path}")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoReceiver()
    window.show()
    sys.exit(app.exec_())