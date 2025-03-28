from PyQt5.QtWidgets import QWidget, QVBoxLayout, QApplication, QPushButton, QHBoxLayout
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import sys
import random

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from Mission_planner.layout.resources.style import control_button_style as st

class AttitudePlotter(QWidget):
    def __init__(self):
        super().__init__()
        self.max_points = 1000

        self.setWindowTitle("Attitude Graph (Pitch, Roll, Yaw)")
        self.setGeometry(300, 200, 1000, 700)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0) 
        main_layout.setSpacing(0)  
        
        # --- Graph ---
        self.pitch_plot = pg.PlotWidget(title="Pitch")
        self.roll_plot = pg.PlotWidget(title="Roll")
        self.yaw_plot = pg.PlotWidget(title="Yaw")

        self.pitch_plot.setBackground("#F3F3E0")
        self.roll_plot.setBackground("#F3F3E0")
        self.yaw_plot.setBackground("#F3F3E0")

        for plot in [self.pitch_plot, self.roll_plot, self.yaw_plot]:
            plot.setYRange(-180, 180)
            plot.showGrid(x=True, y=True)

        main_layout.addWidget(self.pitch_plot)
        main_layout.addWidget(self.roll_plot)
        main_layout.addWidget(self.yaw_plot)

        button_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Tracking")
        self.stop_btn = QPushButton("Stop Tracking")
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.stop_btn)
        main_layout.addLayout(button_layout)
        
        self.start_btn.setStyleSheet(st)
        self.stop_btn.setStyleSheet(st)

        self.setLayout(main_layout)

        self.x = []
        self.pitch_data = []
        self.roll_data = []
        self.yaw_data = []
        self.counter = 0

        self.pitch_curve = self.pitch_plot.plot(self.x, self.pitch_data, pen='#2C3333')
        self.roll_curve = self.roll_plot.plot(self.x, self.roll_data, pen='#2C3333')
        self.yaw_curve = self.yaw_plot.plot(self.x, self.yaw_data, pen='#2C3333')

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)

        self.start_btn.clicked.connect(self.start_tracking)
        self.stop_btn.clicked.connect(self.stop_tracking)

    def start_tracking(self):
        self.timer.start(100)  # cập nhật mỗi 50ms

    def stop_tracking(self):
        self.pitch_data=[]
        self.roll_data=[]
        self.yaw_data=[]
        self.x=[]

        self.timer.stop()

    def update_data(self):
        pitch = random.uniform(-90, 90)
        roll = random.uniform(-90, 90)
        yaw = random.uniform(-180, 180)

        self.x.append(self.counter)
        self.pitch_data.append(pitch)
        self.roll_data.append(roll)
        self.yaw_data.append(yaw)
        self.counter += 1

        # Giới hạn size
        if len(self.x) > self.max_points:
            self.x = self.x[-self.max_points:]
            self.pitch_data = self.pitch_data[-self.max_points:]
            self.roll_data = self.roll_data[-self.max_points:]
            self.yaw_data = self.yaw_data[-self.max_points:]

        self.pitch_curve.setData(self.x, self.pitch_data)
        self.roll_curve.setData(self.x, self.roll_data)
        self.yaw_curve.setData(self.x, self.yaw_data)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = AttitudePlotter()
    viewer.show()
    sys.exit(app.exec_())
