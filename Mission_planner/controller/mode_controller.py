from PyQt5.QtWidgets import QDialog, QRadioButton, QButtonGroup, QLabel, QSpinBox, QPushButton, QMessageBox
from PyQt5.QtCore import QRect

from Mission_planner.communication.pc_mavlink import MAV

class ModeChangeDialog(QDialog):
    current_mode = "Manual"
    current_heading = 0
    current_depth = 0

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Change Mode")
        self.setFixedSize(300, 250)
        
        self._setup_ui()
        self._setup_connections()
        self.load_current_mode()
    
    def _setup_ui(self):
        self.label = QLabel("Select Mode:", self)
        self.label.setGeometry(QRect(20, 20, 100, 30))
        
        self.manual_mode = QRadioButton("Manual", self)
        self.manual_mode.setGeometry(QRect(20, 60, 100, 30))
        
        self.auto_heading_mode = QRadioButton("Auto Heading", self)
        self.auto_heading_mode.setGeometry(QRect(20, 100, 100, 30))
        
        self.auto_depth_mode = QRadioButton("Auto Depth", self)
        self.auto_depth_mode.setGeometry(QRect(20, 140, 100, 30))
        
        self.button_group = QButtonGroup(self)
        self.button_group.addButton(self.manual_mode)
        self.button_group.addButton(self.auto_heading_mode)
        self.button_group.addButton(self.auto_depth_mode)
        
        self.heading_input = QSpinBox(self)
        self.heading_input.setRange(0, 360)
        self.heading_input.setPrefix("Heading: ")
        self.heading_input.setGeometry(QRect(150, 100, 100, 30))
        self.heading_input.hide()
        
        self.heading_range_label = QLabel("Valid range: 0-360°", self)
        self.heading_range_label.setGeometry(QRect(150, 130, 120, 30))
        self.heading_range_label.hide()
        
        self.depth_input = QSpinBox(self)
        self.depth_input.setRange(0, 10)
        self.depth_input.setPrefix("Depth: ")
        self.depth_input.setGeometry(QRect(150, 140, 100, 30))
        self.depth_input.hide()
        
        self.depth_range_label = QLabel("Valid range: 0-10m", self)
        self.depth_range_label.setGeometry(QRect(150, 170, 120, 30))
        self.depth_range_label.hide()
        
        self.change_button = QPushButton("Change", self)
        self.change_button.setGeometry(QRect(100, 210, 100, 30))
    
    def _setup_connections(self):
        self.auto_heading_mode.toggled.connect(self.toggle_heading_input)
        self.auto_depth_mode.toggled.connect(self.toggle_depth_input)
        self.change_button.clicked.connect(self.confirm_change)
    
    def load_current_mode(self):
        if ModeChangeDialog.current_mode == "Manual":
            self.manual_mode.setChecked(True)
            self._hide_all_inputs()
        elif ModeChangeDialog.current_mode == "Auto Heading":
            self.auto_heading_mode.setChecked(True)
            self.heading_input.setValue(ModeChangeDialog.current_heading)
        elif ModeChangeDialog.current_mode == "Auto Depth":
            self.auto_depth_mode.setChecked(True)
            self.depth_input.setValue(ModeChangeDialog.current_depth)

    def _hide_all_inputs(self):
        self.heading_input.hide()
        self.heading_range_label.hide()
        self.depth_input.hide()
        self.depth_range_label.hide()

    def toggle_heading_input(self, checked):
        self.heading_input.setVisible(checked)
        self.heading_range_label.setVisible(checked)
        
        if checked:
            self.depth_input.hide()
            self.depth_range_label.hide()
    
    def toggle_depth_input(self, checked):
        self.depth_input.setVisible(checked)
        self.depth_range_label.setVisible(checked)
        
        if checked:
            self.heading_input.hide()
            self.heading_range_label.hide()
    
    def confirm_change(self):
        reply = QMessageBox.question(
            self, 
            'Confirm Change', 
            'Are you sure you want to change the mode?',
            QMessageBox.Yes | QMessageBox.No, 
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.apply_mode_change()
            self.accept()
    
    def apply_mode_change(self):
        if self.auto_heading_mode.isChecked():
            heading = self.heading_input.value()
            ModeChangeDialog.current_mode = "Auto Heading"
            ModeChangeDialog.current_heading = heading
            MAV.set_auto_heading(heading)
            print(f"Mode: Auto Heading, Heading: {heading}°")
            
        elif self.auto_depth_mode.isChecked():
            depth = self.depth_input.value()
            ModeChangeDialog.current_mode = "Auto Depth"
            ModeChangeDialog.current_depth = depth
            MAV.set_auto_depth(depth)
            print(f"Mode: Auto Depth, Depth: {depth}m")
            
        elif self.manual_mode.isChecked():
            ModeChangeDialog.current_mode = "Manual"
            MAV.set_manual_mode()
            print("Mode: Manual")