from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QApplication
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))) 
#print(os.path.dirname(os.path.abspath(__file__)))

from communication.system_update_timer import SystemStatusManager
from layout.test_widget import widget1,widget2


class BasicWidget(QWidget):
    def __init__(self):
        super().__init__()

        # Tạo layout
        layout = QVBoxLayout()
        self.sys=SystemStatusManager()

        # Thêm một label
        self.label = QLabel("Hello, PyQt5!", self)
        layout.addWidget(self.label)
        
        # Thiết lập layout cho widget
        self.w1=widget1.BasicWidget1(self.sys)
        self.w2=widget2.BasicWidget2(self.sys)

        layout.addWidget(self.w1)
        layout.addWidget(self.w2)

        self.setLayout(layout)
        
        # Thiết lập tiêu đề và kích thước cửa sổ
        self.setWindowTitle("Basic PyQt5 Widget")
        self.resize(300, 200)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BasicWidget()
    window.show()
    sys.exit(app.exec_())
