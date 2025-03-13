from pymavlink import mavutil
import time
import threading

UP = 1000
DOWN = 1001
LEFT = 1002
RIGHT = 1003
FORWARD = 1004
BACKWARD = 1005
STOP = 1006

SET_AUTO_HEADING = 1007
SET_AUTO_DEPTH = 1008
SET_PID = 1009
SET_SPEED_FORWARD = 1021        #Forward speed range is from 1 to 100
SET_SPEED_BACKWARD = 1022       #Backward speed range is from -100 to -1

SET_LIGHT = 1023
SET_CAMERA = 1024

START_MAG_CALIBRATION = 1025

def send_heartbeat(master):
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,           
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(1)

def send_control_cmd(master, cmd):
    master.mav.command_long_send(
        master.target_system,       # ID_SYSTEM
        master.target_component,    # ID_COMPONENT
        cmd,                        # Encoded command
        0,                          # Confirmation - How many commands to send
        0, 0, 0, 0, 0, 0, 0         # Parameters
    )

def set_auto_heading(master, enable, heading):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_AUTO_HEADING,
        0,
        enable, heading, 0, 0, 0, 0, 0
    )

def set_auto_depth(master, enable, depth):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_AUTO_DEPTH,
        0,
        enable, depth, 0, 0, 0, 0, 0
    )

def set_max_speed_forward(master, max_speed):
    if max_speed < 1:
        max_speed = 1
    elif max_speed > 100:
        max_speed = 100
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_SPEED_FORWARD,
        0,
        max_speed, 0, 0, 0, 0, 0, 0
    )

def set_max_speed_backward(master, max_speed):
    if max_speed < -100:
        max_speed = -100
    elif max_speed > -1:
        max_speed = -1
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_SPEED_BACKWARD,
        0,
        max_speed, 0, 0, 0, 0, 0, 0
    )

def set_pid(master, Kp, Ki, Kd):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_PID,
        0,
        Kp, Ki, Kd, 0, 0, 0, 0
    )

def set_light(master, enable):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_LIGHT,
        0,
        enable, 0, 0, 0, 0, 0, 0
    )

def set_camera(master, enable):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        SET_CAMERA,
        0,
        enable, 0, 0, 0, 0, 0, 0
    )

def start_mag_calibration(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        START_MAG_CALIBRATION,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

#list of main parameters that can be requested (see more in status.json):
# - "horizontal_speed"
# - "vertical_speed"
# - "heading"
# - "depth"
# - "battery_status"
# - "temperature"
# - "auto_heading"
# - "auto_depth"
# - "target_heading"
# - "target_depth"
# - "Kp"
# - "Ki"
# - "Kd"
# - "max_speed_forward"
# - "max_speed_backward"
# - "left_speed"
# - "right_speed"
# - "left_depth_speed"
# - "right_depth_speed"
# - "mode"

def get_status(master, status_id):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        status_id.encode('utf-8'),
        -1
    )

def get_all_status(master):
    master.mav.param_request_list_send(
        master.target_system,
        master.target_component
    )

master = mavutil.mavlink_connection("udpout:169.254.54.120:50000")

threading.Thread(target=send_heartbeat, args=(master,), daemon=True).start()

while True:
    set_auto_heading(master, 1, 0)
    time.sleep(1)