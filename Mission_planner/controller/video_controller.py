import os
import sys
import subprocess
import time
import win32gui
import win32process
import win32con
import threading
import queue
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QMessageBox, QLabel
from PyQt5.QtGui import QWindow
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Mission_planner.connection.pc_mavlink import MAV
from Mission_planner.utils.pc_logger import LOG

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
                        LOG.error(f"GStreamer error: {line}")
                        self.error_signal.emit(line)
            except Exception as e:
                LOG.error(f"Error reading GStreamer stderr: {e}")
                break
            time.sleep(0.1)
            
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

def find_window_by_pid(pid, result_queue=None):
    result = []

    LOG.info(f"Finding window for PID: {pid}")
    def callback(hwnd, _):
        if win32gui.IsWindowVisible(hwnd):
            try:
                _, found_pid = win32process.GetWindowThreadProcessId(hwnd)
                if found_pid == pid:
                    title = win32gui.GetWindowText(hwnd)
                    class_name = win32gui.GetClassName(hwnd)
                    LOG.info(f"Found window: {title} (HWND: {hwnd}, PID: {found_pid})")
                    result.append(hwnd)
            except Exception as e:
                LOG.error(f"Error getting window info: {e}")
    
    win32gui.EnumWindows(callback, None)
    
    if not result:
        LOG.info(f"No window found for PID: {pid}")
        return result
    
    if result_queue is not None:
        result_queue.put(result)
    
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
        
        QTimer.singleShot(3000, self.start_gstreamer)
        
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
            
            self.status_label.setText(f"GStreamer started with PID: {self.gst_process.pid}")
            MAV._initialize_connections()
            MAV.start_camera_stream()

            self.process_monitor = ProcessMonitor(self.gst_process)
            self.process_monitor.error_signal.connect(self.handle_gstreamer_error)
            
            QTimer.singleShot(5000, self.find_and_embed_window)
            
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
            LOG.error(error)
            self.status_label.setText(error)
            QMessageBox.critical(self, "Error", error)
            return
        
        self.status_label.setText("Looking for GStreamer window...")
        
        result_queue = queue.Queue()
        
        find_window_thread = threading.Thread(
            target=find_window_by_pid, 
            args=(self.gst_process.pid, result_queue), 
            daemon=True
        )
        find_window_thread.start()
        
        find_window_thread.join(timeout=10)
        
        try:
            hwnds = result_queue.get(block=False)
        except queue.Empty:
            hwnds = []
        
        if not hwnds:
            error = "No GStreamer window found"
            LOG.error(error)
            self.status_label.setText(error)
            return
        
        LOG.info(f"Found GStreamer window: {hwnds[0]}")
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
            LOG.error(error)
            self.status_label.setText(error)
            QMessageBox.critical(self, "Error", error)
    
    def cleanup(self):
        if self.process_monitor:
            self.process_monitor.stop()
        
        if self.gst_process:
            LOG.info("Terminating GStreamer process...")
            try:
                self.gst_process.terminate()
                self.gst_process.wait(timeout=3)
            except Exception as e:
                LOG.error(f"Error terminating GStreamer process: {e}")
                try:
                    self.gst_process.kill()
                except:
                    pass
    
    def closeEvent(self, event):
        self.cleanup()
        return super().closeEvent(event)

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