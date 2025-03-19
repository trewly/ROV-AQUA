from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import canvas từ module
from layout.sub_widgets import canvas, motor_slider
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


app = QApplication([])
window = settingLayout()
window.show()
app.exec_()
