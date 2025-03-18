from .mode_change import ModeChangeDialog
from Mission_planner.communication.pc_mavlink import MAV

def on_surface_button_clicked():
    MAV.send_control_cmd(MAV.SURFACE)
    print("Surface button clicked")

def on_dive_button_clicked():
    MAV.send_control_cmd(MAV.DIVE)
    print("Dive button clicked")

def on_mode_change_button_clicked():
    dialog = ModeChangeDialog()
    dialog.exec_()

def on_light_button_clicked(main_window):
    main_window.light_on = not main_window.light_on
    if main_window.light_on:
        main_window.light_button.setText("LIGHT ON")
        main_window.light_button.setStyleSheet("background-color: yellow; font-weight: bold;")
        print("Light is ON")
    else:
        main_window.light_button.setText("LIGHT OFF")
        main_window.light_button.setStyleSheet("background-color: none; font-weight: normal;")
        print("Light is OFF")