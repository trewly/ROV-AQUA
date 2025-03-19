import sys
import os

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))


from Mission_planner.status import pc_status as status


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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())