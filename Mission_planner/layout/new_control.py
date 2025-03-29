from PyQt5.QtWidgets import QWidget, QPushButton,  QGridLayout, QApplication
from PyQt5.QtGui import QPalette, QColor
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from resources.style import setting_button_style
from layout.sub_widgets import motor_slider

class viewWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(985,785)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#395B64"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        
        #khoi tao slider set toc do motor
        self.motoSlider=motor_slider.MotorSlider()
        self.motoSlider.setParent(self)
        self.motoSlider.setGeometry(280,615,755, 180)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = viewWidget()
    window.show()
    sys.exit(app.exec_())
