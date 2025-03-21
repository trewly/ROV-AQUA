import sys
import os
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QMainWindow, QWidget, QHBoxLayout, QSplitter
from PyQt5.QtCore import Qt

from layout import screen3d
from layout import setting
from layout import control

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ""))) 

class MainWindow(QMainWindow):  # ✅ Sửa QMainwindow → QMainWindow
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AQUA MISSION PLANNER")
        self.setGeometry(0, 0, 1920, 1080)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.main_layout = QHBoxLayout(central_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)  # Xóa khoảng cách mép
        self.main_layout.setSpacing(0)  # Xóa khoảng trống giữa các widget

        self.splitter = QSplitter(Qt.Horizontal)
        self.main_layout.addWidget(self.splitter)

        self.left_screen_init()
        self.right_screen_init()

    def left_screen_init(self):
        self.left_screen = QWidget()
        self.left_screen_layout = QVBoxLayout(self.left_screen)
        self.left_screen_layout.setContentsMargins(0, 0, 0, 0)

        # ✅ Thêm widget hiển thị 3D
        self.up_screen = screen3d.STLViewerWidget("./layout/resources/shell_assem.STL")
        self.left_screen_layout.addWidget(self.up_screen)

        self.down_screen = control.Ui_Form()
        self.left_screen_layout.addWidget(self.down_screen)



        # ✅ Thêm vào splitter
        self.splitter.addWidget(self.left_screen)

    def right_screen_init(self):
        self.right_screen = setting.settingLayout()

        # ✅ Thêm vào splitter
        self.splitter.addWidget(self.right_screen)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
