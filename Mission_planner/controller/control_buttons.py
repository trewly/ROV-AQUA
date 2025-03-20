from .mode_change import ModeChangeDialog
from Mission_planner.communication.pc_mavlink import MAV

from PyQt5.QtCore import Qt

class ButtonController:
    def __init__(self):
        pass

    def on_surface_button_clicked(self):
        MAV.send_control_cmd(MAV.SURFACE)
        print("Surface button clicked")

    def on_surface_button_released(self):
        MAV.send_control_cmd(MAV.STOP)
        print("Surface button released")

    def on_dive_button_clicked(self):
        MAV.send_control_cmd(MAV.DIVE)
        print("Dive button clicked")

    def on_dive_button_released(self):
        MAV.send_control_cmd(MAV.STOP)
        print("Dive button released")

    def on_mode_change_button_clicked(self):
        dialog = ModeChangeDialog()
        dialog.exec_()

    def on_light_button_clicked(self, main_window):
        main_window.light_on = not main_window.light_on
        if main_window.light_on:
            main_window.light_button.setText("LIGHT ON")
            MAV.set_light(1)
            print("Light is ON")
        else:
            main_window.light_button.setText("LIGHT OFF")
            MAV.set_light(0)
            print("Light is OFF")
            
    def on_light_button_released(self):
        pass

    # Các phương thức xử lý sự kiện phím
    def handle_key_press(self, parent_widget, event):
        if event.isAutoRepeat():
            return False

        key = event.key()
        if key == Qt.Key_Shift:
            self.on_surface_button_clicked()
            return True
        elif key == Qt.Key_Control:
            self.on_dive_button_clicked()
            return True
        elif key == Qt.Key_L:
            parent_widget.light_button.click()
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
        elif key == Qt.Key_L:
            return True
        return False


controller = ButtonController()