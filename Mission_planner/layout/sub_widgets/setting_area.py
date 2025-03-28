from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QApplication
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from resources.style import setting_button_style

class settingWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(298,330) 
        self.setStyleSheet("background-color: #395B64;")
        self.settinglayout = QVBoxLayout()  
        
        self.button_init()  

        self.setLayout(self.settinglayout) 

    def button_init(self):
        self.button_1 = QPushButton("Calibration")
        self.button_2 = QPushButton("PID Tuning")
        self.button_3 = QPushButton("System check")
        self.button_4 = QPushButton("Failsafe mechanism")
        self.button_5 = QPushButton("Advanced setting")

        buttons = [self.button_1, self.button_2, self.button_3, self.button_4, self.button_5]
        
        for i, button in enumerate(buttons, start=1):
            button.setStyleSheet(setting_button_style)
            button.setFixedSize(280, 50)
            button.clicked.connect(lambda _, b=i: self.button_clicked(b))
            self.settinglayout.addWidget(button)

    def button_clicked(self, button_number):
        print(f"Button {button_number} clicked")

