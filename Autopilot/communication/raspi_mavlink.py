import time
import threading
import sys
import os
import socket

from pymavlink import mavutil

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))


from Autopilot.system_info.status import raspi_status as status
from Autopilot.system_info.sensor import raspi_sensor_calibrate as calibrate
from Autopilot.control.motor import raspi_motor_control as rov

last_hearbeat = 0

SURFACE = 1000
DIVE = 1001
LEFT = 1002
RIGHT = 1003
FORWARD = 1004
BACKWARD = 1005
STOP = 1006

SET_MANUAL = 1100
SET_AUTO_HEADING = 1101
SET_AUTO_DEPTH = 1102
SET_PID = 1103
SET_SPEED_FORWARD = 1104        # Forward speed range is from 1 to 100
SET_SPEED_BACKWARD = 1105       # Backward speed range is from -100 to -1

SET_LIGHT = 1106
SET_CAMERA = 1107

START_MAG_CALIBRATION = 1200

def handle_received_msg(msg):
    if msg == None:
        return

    if msg.get_type() == "COMMAND_LONG":
        if status.read_status(key="mode") == "manual":
            if msg.command == SURFACE:
                print("SURFACE")
                rov.surface()
            elif msg.command == DIVE:
                print("DIVE")
                rov.dive()
            elif msg.command == LEFT:
                print("LEFT")
                rov.turn_left()
            elif msg.command == RIGHT:
                print("RIGHT")
                rov.turn_right()
            elif msg.command == FORWARD:
                print("FORWARD")
                rov.move_forward()
            elif msg.command == BACKWARD:
                print("BACKWARD")
                rov.move_backward()
            elif msg.command == STOP:
                print("STOP")
                rov.stop_all()
        
        if msg.command == SET_MANUAL:
            status.update_status(key="mode", value="manual")
            status.update_status(key="auto_heading", value=False)
            status.update_status(key="auto_depth", value=False)
            print("Manual mode set")

        elif msg.command == SET_AUTO_HEADING:
            status.update_status(key="auto_heading", value=msg.param1)
            status.update_status(key="target_heading", value=msg.param2)
            status.update_status(key="mode", value="auto_heading")
            print("Auto heading set to: ", msg.param1, msg.param2)

        elif msg.command == SET_AUTO_DEPTH:
            status.update_status(key="auto_depth", value=msg.param1)
            status.update_status(key="target_depth", value=msg.param2)
            status.update_status(key="mode", value="auto_depth")
            print("Auto depth set to: ", msg.param1, msg.param2)

        elif msg.command == SET_PID:
            status.update_status(key="Kp", value=msg.param1)
            status.update_status(key="Ki", value=msg.param2)
            status.update_status(key="Kd", value=msg.param3)
            print("PID set to: ", msg.param1, msg.param2, msg.param3)

        elif msg.command == SET_SPEED_FORWARD:
            status.update_status(key="max_speed_forward", value=msg.param1)
            print("Speed set to: ", msg.param1)

        elif msg.command == SET_SPEED_BACKWARD:
            status.update_status(key="max_speed_backward", value=msg.param1)
            print("Speed set to: ", msg.param1)
        
        elif msg.command == SET_LIGHT:
            status.update_status(key="light", value=msg.param1)
            print("Light set to: ", msg.param1)

        elif msg.command == SET_CAMERA:
            status.update_status(key="camera", value=msg.param1)
            print("Camera set to: ", msg.param1)
        
        elif msg.command == START_MAG_CALIBRATION:
            calibrate.calibrate_mag()

def received_msg(master):
    while True:
        msg = master.recv_match(blocking=True)
        if msg.get_type() == "HEARTBEAT":
            last_hearbeat = time.time()

        elif time.time() - last_hearbeat > 5:
            print("Connection lost")
            continue
        handle_received_msg(msg)


def send_status(master):
    while True:
        status_data = status.read_all_status()
        
        for key in ["roll", "pitch", "heading", "temp", "depth"]:
            try:
                value = float(status_data.get(key, 0))
                param_id = key.ljust(16, '\0')
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                
                master.mav.param_value_send(
                    param_id.encode("ascii"),
                    value,
                    param_type,
                    0,
                    4
                )
                print(f"Sent {key}: {value}")
            except Exception as e:
                print(f"Error sending {key}: {e}")
            time.sleep(0.01)
        
        time.sleep(1)

master_receive = mavutil.mavlink_connection("udpin:0.0.0.0:50000")
thread1 = threading.Thread(target=received_msg, args=(master_receive,), daemon=True)
thread1.start()

master_send = mavutil.mavlink_connection("udpout:169.254.54.121:50001")
thread2 = threading.Thread(target=send_status, args=(master_send,), daemon=True)
thread2.start()

while True:
    time.sleep(1)