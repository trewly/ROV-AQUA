from PyQt5.QtCore import Qt

from .mode_controller import ModeChangeDialog
from Mission_planner.connection.pc_mavlink import MAV

class ButtonController:
    KEY_MAPPINGS = {
        Qt.Key_Shift: 'surface',
        Qt.Key_Control: 'dive',
        Qt.Key_R: 'roll_right',
        Qt.Key_F: 'roll_left',
    }
    
    def __init__(self):
        self.active_buttons = set()
    
    def on_button_clicked(self, button_type):
        self.active_buttons.add(button_type)
        
        if button_type == 'surface':
            MAV.send_control_cmd(MAV.SURFACE)
        
        elif button_type == 'dive':
            MAV.send_control_cmd(MAV.DIVE)
        
        elif button_type == 'roll_right':
            MAV.send_control_cmd(MAV.ROLL_RIGHT)

        elif button_type == 'roll_left':
            MAV.send_control_cmd(MAV.ROLL_LEFT)
    
    def on_button_released(self, button_type):
        if button_type in self.active_buttons:
            self.active_buttons.remove(button_type)
        
        if button_type in ('surface', 'dive', 'roll_right', 'roll_left'):
            MAV.send_control_cmd(MAV.STOP)
    
    def on_calib_button_clicked(self):
        MAV.send_control_cmd(MAV.CALIBRATE)
        print("Calibrate button clicked")

    def on_start_button_clicked(self):
        MAV._initialize_connections()
        print("Start button clicked")

    def on_stop_button_clicked(self):
        MAV.shutdown()
        print("Stop button clicked")

    def on_exit_button_clicked(self):
        MAV.shutdown()
        print("Exit button clicked")
        
    def on_surface_button_clicked(self):
        self.on_button_clicked('surface')
        print("Surface button clicked")

    def on_surface_button_released(self):
        self.on_button_released('surface')
        print("Surface button released")

    def on_dive_button_clicked(self):
        self.on_button_clicked('dive')
        print("Dive button clicked")

    def on_dive_button_released(self):
        self.on_button_released('dive')
        print("Dive button released")

    def on_roll_right_button_clicked(self):
        self.on_button_clicked('roll_right')
        print("Roll right button clicked")

    def on_roll_right_button_released(self):
        self.on_button_released('roll_right')
        print("Roll right button released")
    
    def on_roll_left_button_clicked(self):
        self.on_button_clicked('roll_left')
        print("Roll left button clicked")
    
    def on_roll_left_button_released(self):
        self.on_button_released('roll_left')
        print("Roll left button released")

    def on_mode_change_button_clicked(self):
        dialog = ModeChangeDialog()
        dialog.exec_()

    def handle_key_press(self, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key == Qt.Key_Shift:
            self.on_surface_button_clicked()
            return True
        elif key == Qt.Key_Control:
            self.on_dive_button_clicked()
            return True
        elif key == Qt.Key_R:
            self.on_roll_right_button_clicked()
            return True
        elif key == Qt.Key_F:
            self.on_roll_left_button_clicked()
            return True
        return False

    def handle_key_release(self, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key == Qt.Key_Shift:
            self.on_surface_button_released()
            return True
        elif key == Qt.Key_Control:
            self.on_dive_button_released()
            return True
        elif key == Qt.Key_R:
            self.on_roll_right_button_released()
            return True
        elif key == Qt.Key_F:
            self.on_roll_left_button_released()
            return True
        return False

controller = ButtonController()