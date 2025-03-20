import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QSplitter
from PyQt5.QtCore import Qt

from layout import screen3d
from layout import setting

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ""))) 

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AQUA MISSION PLANER")
        self.setGeometry(0, 0, 1920, 1080)

        # Tạo layout chính
        self.main_layout = QHBoxLayout(self)
        self.setLayout(self.main_layout)

        # Sử dụng QSplitter để chia giao diện
        self.splitter = QSplitter(Qt.Horizontal)
        self.main_layout.addWidget(self.splitter)

        # Màn hình bên trái (hiển thị 3D)
        self.left_screen_init()

        # Màn hình bên phải (bảng điều khiển)
        self.right_screen_init()

    def left_screen_init(self):
        self.left_screen = QWidget()
        self.left_screen_layout = QHBoxLayout(self.left_screen)

        self.up_screen = screen3d.STLViewerWidget("./layout/resources/shell_assem.STL")

        self.left_screen_layout.addWidget(self.up_screen)
        self.splitter.addWidget(self.left_screen)

    def right_screen_init(self):
        self.right_screen = setting.settingLayout()
        self.splitter.addWidget(self.right_screen)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
