from pymavlink import mavutil
import time
import threading
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import Mission_planner.status.pc_status as status

class MavlinkController:
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
    SET_SPEED_DIVE = 1106
    SET_SPEED_SURFACE = 1107

    SET_LIGHT = 1106
    SET_CAMERA = 1107

    START_MAG_CALIBRATION = 1200

    def __init__(self):
        self.master_send = mavutil.mavlink_connection("udpout:169.254.54.120:5000")
        send_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        send_thread.start()

        self.master_receive = mavutil.mavlink_connection("udpin:0.0.0.0:5001")
        receive_thread = threading.Thread(target=self.receive_msg, daemon=True)
        receive_thread.start()

        self.timer = time.time()

    def send_heartbeat(self):
        while True:
            self.master_send.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(3)

    def receive_msg(self):
        while True:
            try:
                msg = self.master_receive.recv_match(blocking=True)
                if msg is not None:
                    self.timer = time.time()
                    if msg.get_type() == "PARAM_VALUE":
                        param_id = msg.param_id.rstrip('\x00')
                        param_value = msg.param_value
                        status.update_status(param_id, param_value)
                        print(f"Received: {param_id} = {param_value}")
                if time.time() - self.timer > 5:
                    print("Connection lost")
                    status.update_status("disconnect", True)
            except Exception as e:
                print(f"Error receiving message: {e}")

    def send_control_cmd(self, cmd):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,       # ID_SYSTEM
            self.master_send.target_component,    # ID_COMPONENT
            cmd,                             # Encoded command
            0,                               # Confirmation - How many commands to send
            0, 0, 0, 0, 0, 0, 0              # Parameters
        )

    def set_manual_mode(self):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_MANUAL,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def set_auto_heading(self, heading):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_AUTO_HEADING,
            0,
            heading, 0, 0, 0, 0, 0, 0
        )

    def set_auto_depth(self, depth):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_AUTO_DEPTH,
            0,
            depth, 0, 0, 0, 0, 0, 0
        )

    def set_max_speed_forward(self, max_speed):
        if max_speed < 1:
            max_speed = 1
        elif max_speed > 100:
            max_speed = 100
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_SPEED_FORWARD,
            0,
            max_speed, 0, 0, 0, 0, 0, 0
        )

    def set_max_speed_backward(self, max_speed):
        if max_speed < -100:
            max_speed = -100
        elif max_speed > -1:
            max_speed = -1
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_SPEED_BACKWARD,
            0,
            max_speed, 0, 0, 0, 0, 0, 0
        )

    def set_max_speed_dive(self, max_speed):
        if max_speed < -100:
            max_speed = -100
        elif max_speed > -1:
            max_speed = -1
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_SPEED_DIVE,
            0,
            max_speed, 0, 0, 0, 0, 0, 0
        )
    
    def set_max_speed_surface(self, max_speed):
        if max_speed < 1:
            max_speed = 1
        elif max_speed > 100:
            max_speed = 100
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_SPEED_SURFACE,
            0,
            max_speed, 0, 0, 0, 0, 0, 0
        )
        
    def set_pid(self, Kp, Ki, Kd):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_PID,
            0,
            Kp, Ki, Kd, 0, 0, 0, 0
        )

    def set_light(self, enable):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_LIGHT,
            0,
            enable, 0, 0, 0, 0, 0, 0
        )

    def set_camera(self, enable):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.SET_CAMERA,
            0,
            enable, 0, 0, 0, 0, 0, 0
        )

    def start_mag_calibration(self):
        self.master_send.mav.command_long_send(
            self.master_send.target_system,
            self.master_send.target_component,
            self.START_MAG_CALIBRATION,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def get_status(self, status_id):
        self.master_send.mav.param_request_read_send(
            self.master_send.target_system,
            self.master_send.target_component,
            status_id.encode('utf-8'),
            -1
        )

MAV = MavlinkController()