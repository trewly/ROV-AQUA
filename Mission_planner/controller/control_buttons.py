from .mode_change import ModeChangeDialog

def on_surface_button_clicked():
    print("Surface button clicked")

def on_dive_button_clicked():
    print("Dive button clicked")

def on_mode_change_button_clicked():
    dialog = ModeChangeDialog()
    dialog.exec_()

def on_light_button_clicked(self):
    self.light_on = not self.light_on
    if self.light_on:
        self.light_button.setText("LIGHT ON")
        self.light_button.setStyleSheet("background-color: yellow; font-weight: bold;")
        print("Light is ON")
    else:
        self.light_button.setText("LIGHT OFF")
        self.light_button.setStyleSheet("background-color: none; font-weight: normal;")
        print("Light is OFF")