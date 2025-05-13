from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QHBoxLayout, QGroupBox, QGridLayout, QPushButton
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor
import sys

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Mission_planner.connection.pc_mavlink import MAV
from Mission_planner.status import pc_status as status
from Mission_planner.config import raspi_config as config
from Mission_planner.layout.resources import style as st

class PIDTuner(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Tuner")
        self.setGeometry(100, 100, 1200, 400)
        
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#F3F3E0")) 
        self.setPalette(palette)
        self.setAutoFillBackground(True)

        layout = QVBoxLayout()
        
        self.pid_params = {
            "autoheading": [1.0, 0.1, 0.01],
            "yaw": [1.0, 0.1, 0.01],
            "depth": [1.0, 0.1, 0.01]
        }
        
        #doc lan dau
        self.initial_read_value()

        self.updated_params = {k: v.copy() for k, v in self.pid_params.items()}
        self.sliders = {}
        
        control_layout = QHBoxLayout()

        #tao slider
        for pid_name in self.pid_params:
            control_layout.addWidget(self.create_pid_controls(pid_name)) 
        layout.addLayout(control_layout)
        
        #tao button
        button_layout = QHBoxLayout()
        for pid_name in self.pid_params:
            button = QPushButton(f"Update {pid_name}")
            button.clicked.connect(lambda checked, n=pid_name: self.confirm_update(n))
            button_layout.addWidget(button)
            button.setStyleSheet(st.canvas_button_style)
        layout.addLayout(button_layout)
        
        #tao reset button
        reset_button = QPushButton("Reset")
        reset_button.clicked.connect(self.reset_values)
        reset_button.setStyleSheet(st.control_button_style)
        layout.addWidget(reset_button)
        
        self.setLayout(layout)
        
    def create_pid_controls(self, pid_name):
        group_box = QGroupBox(pid_name)
        group_box.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 20px;
                border: 2px solid #395B64;
                border-radius: 5px;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 2px 10px;
                color: #395B64;
            }
        """)
        grid = QGridLayout()
        
        labels = ['Kp', 'Ki', 'Kd']
        ranges = {'Kp': (0.1, 100), 'Ki': (0.001, 10), 'Kd': (0.001, 10)}
        steps = {'Kp': 0.5, 'Ki': 0.001, 'Kd': 0.001}
        
        self.sliders[pid_name] = {}
        
        for i, param in enumerate(labels):
            label = QLabel(f"{param}: {self.pid_params[pid_name][i]:.3f}")
            slider = QSlider(Qt.Horizontal)

            label.setStyleSheet("font-size: 16px;")

            min_val, max_val = ranges[param]
            step = steps[param]
            
            slider.setMinimum(int(min_val * (1/step)))
            slider.setMaximum(int(max_val * (1/step)))
            slider.setSingleStep(1)
            slider.setValue(int(self.pid_params[pid_name][i] * (1/step)))
            
            slider.valueChanged.connect(lambda value, p=param, n=pid_name, l=label, s=step: self.update_temp_pid(n, p, value, l, s))
            
            grid.addWidget(label, i, 0)
            grid.addWidget(slider, i, 1)
            self.sliders[pid_name][param] = slider
        
        group_box.setLayout(grid)
        return group_box
        #Kp_autoheading
        
    def initial_read_value(self):
        pid = ['Kp', 'Ki', 'Kd']
        for param, values in self.pid_params.items():
            for index, k in enumerate(pid):
                self.pid_params[param][index] = config.read_config(f"{k}_{param}")
                print(self.pid_params)

    def update_temp_pid(self, pid_name, param, value, label, step):
        index = {'Kp': 0, 'Ki': 1, 'Kd': 2}[param]
        self.updated_params[pid_name][index] = round(value * step, 3)
        label.setText(f"{param}: {self.updated_params[pid_name][index]:.3f}")
    
    def confirm_update(self, pid_name):
        self.pid_params[pid_name] = self.updated_params[pid_name].copy()
        print(f"PID parameters for {pid_name} updated:", self.pid_params[pid_name])
        # update and transmit
        if pid_name in ["autoheading", "depth", "yaw"]:  
            # write json
            status.update_status(f"Kp_{pid_name}", self.pid_params[pid_name][0])
            status.update_status(f"Ki_{pid_name}", self.pid_params[pid_name][1])
            status.update_status(f"Kd_{pid_name}", self.pid_params[pid_name][2])
            # transmit
            if pid_name == "autoheading":
                MAV.set_pid_autoheading(self.pid_params[pid_name][0], self.pid_params[pid_name][1], self.pid_params[pid_name][2])
            elif pid_name == "depth":
                MAV.set_pid_depth(self.pid_params[pid_name][0], self.pid_params[pid_name][1], self.pid_params[pid_name][2])
            elif pid_name == "yaw":
                MAV.set_pid_yaw(self.pid_params[pid_name][0], self.pid_params[pid_name][1], self.pid_params[pid_name][2])

    def reset_values(self):
        self.updated_params = {k: v.copy() for k, v in self.pid_params.items()}
        for pid_name, params in self.updated_params.items():
            for i, param in enumerate(['Kp', 'Ki', 'Kd']):
                step = 0.5 if param == 'Kp' else 0.001
                self.sliders[pid_name][param].setValue(int(params[i] * (1/step)))
        self.update()

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     window = PIDTuner()
#     window.show()
#     sys.exit(app.exec_())