from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtGui import QPalette, QColor
import sys

class testLayout(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(960,540)  # Đặt kích thước cố định
        self.setWindowTitle("#F3F3E0")

        # Đặt màu nền xanh lá
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#F3F3E0"))  # Màu xanh lá
        self.setPalette(palette)
        self.setAutoFillBackground(True)

