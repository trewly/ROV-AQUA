import sys
import subprocess
import time
import win32gui
import win32process
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtGui import QWindow
from PyQt5.QtWidgets import QHBoxLayout

def find_window_by_pid(pid):
    result = []

    def callback(hwnd, _):
        _, found_pid = win32process.GetWindowThreadProcessId(hwnd)
        if found_pid == pid and win32gui.IsWindowVisible(hwnd):
            result.append(hwnd)

    win32gui.EnumWindows(callback, None)
    return result

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt + GStreamer Video Dock")
        self.resize(800, 600)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Start GStreamer
        self.gst_process = subprocess.Popen([
            "D:\\gstreamer\\1.0\\msvc_x86_64\\bin\\gst-launch-1.0.exe",
            "videotestsrc", "!", "glimagesink", "sync=false"
        ])

        # Tìm cửa sổ sau 1-2 giây
        time.sleep(2)

        hwnds = find_window_by_pid(self.gst_process.pid)
        if not hwnds:
            print("❗️ Không tìm thấy cửa sổ GStreamer.")
            return
        print(f"✅ Found window: {hwnds[0]}")

        # Dock vào PyQt
        window = QWindow.fromWinId(hwnds[0])
        container = QWidget.createWindowContainer(window)
        self.layout.addWidget(container)

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
