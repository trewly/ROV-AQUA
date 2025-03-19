from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt, QThread, pyqtSignal

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import canvas từ module
from layout.sub_widgets import canvas, motor_slider
from Mission_planner.status import pc_status as status
# import sys
# import os

# print("Current sys.path:", sys.path)

class settingLayout(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(540,960)
        layout = QVBoxLayout()  # Layout chính
        
        #khoi tao view widget
        self.view_init()

        #khoi tao canvas
        self.canvas_init()

        #khoi tao bar thong so
        self.info_bar_init()

        #khoi tao bar setting
        self.setting_bar_init()

        self.setLayout(layout)  # Đặt layout cho widget
    def view_init(self):
        pass
    def canvas_init(self):
        pass 
    def info_bar_init(self):
        pass
    def setting_bar_init(self):
        pass


class StatusReaderThread(QThread):
    status_signal = pyqtSignal(str, object)

    def __init__(self, key, parent=None):
        super().__init__(parent)
        self.key = key
        self.running = True

    def run(self):
        while self.running:
            try:
                value = status.read_status(self.key)
                self.status_signal.emit(self.key, value)
            except Exception as e:
                print(f"Error reading status: {e}")

    def stop(self):
        self.running = False
        self.wait()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Status Reader")
        self.resize(400, 200)

        self.label = QLabel("Waiting for status...", self)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.status_thread = StatusReaderThread(key="depth")
        self.status_thread.status_signal.connect(self.update_label)
        self.status_thread.start()

    def update_label(self, key, value):
        self.label.setText(f"{key}: {value}")

    def closeEvent(self, event):
        self.status_thread.stop()
        event.accept()


app = QApplication([])
window = settingLayout()
window.show()
app.exec_()
