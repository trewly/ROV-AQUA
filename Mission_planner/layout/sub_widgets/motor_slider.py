from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QSlider,
    QGridLayout, QSizePolicy, QVBoxLayout
)
from PyQt5.QtCore import Qt

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Mission_planner.communication.pc_mavlink import MAV
from Mission_planner.layout.resources import style as st


class MotorSlider(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor speed testing")
        self.setFixedSize(700, 180)
        self.setStyleSheet("background-color: #395B64;")

        # Layout chính
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)

        # Tạo layout con cho xy và z
        self.xy_speed_widget = QWidget()
        self.z_speed_widget = QWidget()

        self.xy_layout = QGridLayout()
        self.z_layout = QGridLayout()

        self.xy_speed_widget.setLayout(self.xy_layout)
        self.z_speed_widget.setLayout(self.z_layout)

        self.xy_speed_set()
        self.z_speed_set()

        main_layout.addWidget(self.xy_speed_widget)
        main_layout.addWidget(self.z_speed_widget)
        self.setLayout(main_layout)

    def xy_speed_set(self):
        # Label
        self.label1 = QLabel("Motor xy: 50/50")
        self.label1.setStyleSheet(st.normal_text_info_style)

        # Sliders
        self.motorxy_forward = QSlider(Qt.Horizontal)
        self.motorxy_backward = QSlider(Qt.Horizontal)
        self.setup_slider(self.motorxy_forward, self.update_label1)
        self.setup_slider(self.motorxy_backward, self.update_label1)

        # Add to layout
        self.xy_layout.addWidget(self.label1, 0, 0)
        self.xy_layout.addWidget(self.motorxy_forward, 0, 1)
        self.xy_layout.addWidget(self.motorxy_backward, 1, 1)

    def z_speed_set(self):
        # Label
        self.label2 = QLabel("Motor z: 50/50")
        self.label2.setStyleSheet(st.normal_text_info_style)

        # Sliders
        self.motorz_forward = QSlider(Qt.Horizontal)
        self.motorz_backward = QSlider(Qt.Horizontal)
        self.setup_slider(self.motorz_forward, self.update_label2)
        self.setup_slider(self.motorz_backward, self.update_label2)

        # Add to layout
        self.z_layout.addWidget(self.label2, 0, 0)
        self.z_layout.addWidget(self.motorz_forward, 0, 1)
        self.z_layout.addWidget(self.motorz_backward, 1, 1)

    def setup_slider(self, slider, slot):
        slider.setMinimum(0)
        slider.setMaximum(100)
        slider.setValue(50)
        slider.setFixedWidth(450)
        slider.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        slider.sliderReleased.connect(slot)

    def update_label1(self):
        sender = self.sender()
        if sender == self.motorxy_forward:
            MAV.set_max_speed_forward(self.motorxy_forward.value())
        elif sender == self.motorxy_backward:
            MAV.set_max_speed_backward(self.motorxy_backward.value())
        self.label1.setText(f"Motor xy: {self.motorxy_forward.value()}/{self.motorxy_backward.value()}")

    def update_label2(self):
        sender = self.sender()
        if sender == self.motorz_forward:
            MAV.set_max_speed_surface(self.motorz_forward.value())
        elif sender == self.motorz_backward:
            MAV.set_max_speed_dive(self.motorz_backward.value())
        self.label2.setText(f"Motor z: {self.motorz_forward.value()}/{self.motorz_backward.value()}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorSlider()
    window.show()
    sys.exit(app.exec_())
