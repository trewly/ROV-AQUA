import sys
import cv2
import numpy as np
import time
import threading
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QObject, pyqtSignal, Qt

class VideoWorkerSignals(QObject):
    frame_ready = pyqtSignal(np.ndarray)
    connection_changed = pyqtSignal(bool)

class VideoWorker:
    def __init__(self, udp_ip, udp_port):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.running = True
        self.connected = False
        
        self.thread = None
        
        self.signals = VideoWorkerSignals()
        
    def start_worker(self):
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
        
    def run(self):
        cap = None
        
        while self.running:
            try:
                if cap is None or not cap.isOpened():
                    stream_url = f"udp://{self.udp_ip}:{self.udp_port}?overrun_nonfatal=1&fifo_size=500"
                    
                    print(f"Creating connection to {stream_url}")
                    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
                    
                    if not cap.isOpened():
                        print("Failed to open video stream")
                        time.sleep(0.5)
                        continue
                    
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FPS, 24)
                
                ret, frame = cap.read()
                
                if not ret or frame is None or frame.size == 0:
                    print("Error reading frame")
                    time.sleep(0.01)
                    continue
                
                if not self.connected:
                    self.connected = True
                    self.signals.connection_changed.emit(True)
                
                try:
                    frame = cv2.flip(frame, -1)
                    self.signals.frame_ready.emit(frame)
                except Exception as e:
                    print(f"Error processing frame: {e}")
                    
            except Exception as e:
                print(f"Video worker error: {e}")
                time.sleep(0.1)
                
                if self.connected:
                    self.connected = False
                    self.signals.connection_changed.emit(False)
                
                if cap is not None:
                    cap.release()
                    cap = None

        if cap is not None:
            cap.release()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3.0)

class VideoReceiver(QWidget):
    def __init__(self, udp_ip="169.254.54.121", udp_port=5000, parent=None):
        super().__init__(parent)
        self.setGeometry(0, 0, 960, 540)
        
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 960, 540)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setText("Connecting to video stream...")
        self.label.setStyleSheet("background-color: black; color: white; font-size: 20px;")
        
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        
        self.worker = VideoWorker(udp_ip, udp_port)
        self.worker.signals.frame_ready.connect(self.update_frame)  
        self.worker.signals.connection_changed.connect(self.update_connection_status)
        self.worker.start_worker()
        
        self.frame_count = 0
        self.fps = 0
        self.fps_timer = time.time()
        
    def update_frame(self, frame):
        try:
            if frame is None or frame.size == 0:
                return
            
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            height, width, channel = frame_rgb.shape
            bytes_per_line = 3 * width
            qimg = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            
            self.label.setPixmap(pixmap)
                        
        except Exception as e:
            print(f"Error updating UI: {e}")

    def update_connection_status(self, connected):
        if connected:
            self.label.clear()
        else:
            self.label.setText("Connection lost. Reconnecting...")
            
    def closeEvent(self, event):
        self.worker.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoReceiver()
    window.show()
    sys.exit(app.exec_())