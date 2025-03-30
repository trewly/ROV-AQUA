import sys
import subprocess
import time
import win32gui
import win32process
import win32con
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QMessageBox, QLabel
from PyQt5.QtGui import QWindow
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject

from Mission_planner.communication.pc_mavlink import MAV

class ProcessMonitor(QObject):
    error_signal = pyqtSignal(str)
    
    def __init__(self, process):
        super().__init__()
        self.process = process
        self.running = True
        self.thread = threading.Thread(target=self._monitor_process)
        self.thread.daemon = True
        self.thread.start()
        
    def _monitor_process(self):
        while self.running and self.process and self.process.poll() is None:
            try:
                stderr_line = self.process.stderr.readline()
                if stderr_line:
                    line = stderr_line.strip()
                    if line and "ERROR" in line:
                        print(f"GStreamer error: {line}")
                        self.error_signal.emit(line)
            except Exception as e:
                print(f"Error reading process output: {e}")
            time.sleep(0.1)
            
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

def find_window_by_pid(pid, max_attempts=5):
    result = []
    attempt = 0
    
    while attempt < max_attempts and not result:
        attempt += 1
        print(f"Searching for GStreamer window (attempt {attempt}/{max_attempts})...")
        
        def callback(hwnd, _):
            if win32gui.IsWindowVisible(hwnd):
                try:
                    _, found_pid = win32process.GetWindowThreadProcessId(hwnd)
                    if found_pid == pid:
                        title = win32gui.GetWindowText(hwnd)
                        class_name = win32gui.GetClassName(hwnd)
                        print(f"Found window: HWND={hwnd}, Title='{title}', Class='{class_name}'")
                        result.append(hwnd)
                except Exception as e:
                    print(f"Error during window enumeration: {e}")
        
        win32gui.EnumWindows(callback, None)
        
        if not result:
            time.sleep(1)
            
    return result

class VideoReceiver(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(960, 540)

        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        self.status_label = QLabel("Initializing video...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: white; background-color: black;")
        self.layout.addWidget(self.status_label)
        
        self.window_find_attempts = 0
        self.gst_process = None
        self.process_monitor = None
        
        QTimer.singleShot(500, self.start_gstreamer)
        
    def start_gstreamer(self):
        GST_PATH = r"C:\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe"
        
        try:
            self.status_label.setText("Starting GStreamer...")
            
            self.gst_process = subprocess.Popen(
                [
                    GST_PATH,
                    "-v",
                    "udpsrc",
                    "port=5000",
                    "!", "tsparse",
                    "!", "tsdemux",
                    "!" , "h264parse",
                    "!", "avdec_h264",
                    "!", "autovideosink",
                    "sync=false"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            print(f"GStreamer process started with PID: {self.gst_process.pid}")
            self.status_label.setText(f"GStreamer started with PID: {self.gst_process.pid}")
            MAV.start_camera_stream()
            self.process_monitor = ProcessMonitor(self.gst_process)
            self.process_monitor.error_signal.connect(self.handle_gstreamer_error)
            
            QTimer.singleShot(2000, self.find_and_embed_window)
            
        except Exception as e:
            error = f"Failed to start GStreamer: {e}"
            print(f"❗️ {error}")
            self.status_label.setText(error)
            QMessageBox.critical(self, "Error", error)
    
    def handle_gstreamer_error(self, error_message):
        self.status_label.setText(f"GStreamer error: {error_message}")
    
    def find_and_embed_window(self):
        if not self.gst_process or self.gst_process.poll() is not None:
            error = "GStreamer process is not running"
            print(f"❗️ {error}")
            self.status_label.setText(error)
            QMessageBox.critical(self, "Error", error)
            return
        
        self.status_label.setText("Looking for GStreamer window...")
        
        hwnds = threading.Thread(target=find_window_by_pid, args=(self.gst_process.pid,), daemon=True).start()
        
        if not hwnds:
            error = "Không tìm thấy cửa sổ GStreamer"
            print(f"❗️ {error}")
            self.status_label.setText(error)
            
            self.window_find_attempts += 1
            
            if self.window_find_attempts <= 3:
                self.status_label.setText(f"Retrying window search (attempt {self.window_find_attempts}/3)...")
                QTimer.singleShot(2000, self.find_and_embed_window)
            else:
                QMessageBox.warning(self, "Warning", "Could not find GStreamer window after multiple attempts.")
            return
            
        print(f"✅ Found window: {hwnds[0]}")
        self.status_label.setText(f"Found window: {hwnds[0]}")
        
        try:
            win32gui.ShowWindow(hwnds[0], win32con.SW_SHOW)
            
            window = QWindow.fromWinId(hwnds[0])
            container = QWidget.createWindowContainer(window)
            
            container.setFocusPolicy(Qt.StrongFocus)
            container.setMinimumSize(640, 360)
            
            self.layout.removeWidget(self.status_label)
            self.status_label.setVisible(False)
            
            self.layout.addWidget(container)
            
        except Exception as e:
            error = f"Error embedding GStreamer window: {e}"
            print(f"❗️ {error}")
            self.status_label.setText(error)
            QMessageBox.critical(self, "Error", error)
    
    def cleanup(self):
        if self.process_monitor:
            self.process_monitor.stop()
        
        if self.gst_process:
            print("Terminating GStreamer process...")
            try:
                self.gst_process.terminate()
                self.gst_process.wait(timeout=3)
            except Exception as e:
                print(f"Error terminating GStreamer: {e}")
                try:
                    self.gst_process.kill()
                except:
                    pass
    
    def closeEvent(self, a0):
        self.cleanup()
        return super().closeEvent(a0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    main_window = QMainWindow()
    main_window.setWindowTitle("Video Test")
    main_window.resize(980, 560)
    
    video_widget = VideoReceiver()
    main_window.setCentralWidget(video_widget)
    
    app.aboutToQuit.connect(video_widget.cleanup)
    
    main_window.show()
    sys.exit(app.exec_())