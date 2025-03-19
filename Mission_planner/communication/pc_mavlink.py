from pymavlink import mavutil
import time
import threading

import Mission_planner.status.pc_status as status

class MavlinkController:
    SURFACE = 1000
    DIVE = 1001
    LEFT = 1002
    RIGHT = 1003
    FORWARD = 1004
    BACKWARD = 1005
    STOP = 1006

    SET_MANUAL = 1111
    SET_AUTO_HEADING = 1007
    SET_AUTO_DEPTH = 1008
    SET_PID = 1009
    SET_SPEED_FORWARD = 1021        # Forward speed range is from 1 to 100
    SET_SPEED_BACKWARD = 1022       # Backward speed range is from -100 to -1

    SET_LIGHT = 1023
    SET_CAMERA = 1024

    START_MAG_CALIBRATION = 1025

    def __init__(self):
        self.master = mavutil.mavlink_connection("udpout:169.254.54.120:50000")
        threading.Thread(target=self.send_heartbeat, daemon=True).start()

    def send_heartbeat(self):
        while True:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(3)

    def send_control_cmd(self, cmd):
        self.master.mav.command_long_send(
            self.master.target_system,       # ID_SYSTEM
            self.master.target_component,    # ID_COMPONENT
            cmd,                             # Encoded command
            0,                               # Confirmation - How many commands to send
            0, 0, 0, 0, 0, 0, 0              # Parameters
        )

    def set_manual_mode(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_MANUAL,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def set_auto_heading(self, enable, heading):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_AUTO_HEADING,
            0,
            enable, heading, 0, 0, 0, 0, 0
        )
        print("Sending")

    def set_auto_depth(self, enable, depth):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_AUTO_DEPTH,
            0,
            enable, depth, 0, 0, 0, 0, 0
        )

    def set_max_speed_forward(self, max_speed):
        if max_speed < 1:
            max_speed = 1
        elif max_speed > 100:
            max_speed = 100
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_SPEED_FORWARD,
            0,
            max_speed, 0, 0, 0, 0, 0, 0
        )

    def set_max_speed_backward(self, max_speed):
        if max_speed < -100:
            max_speed = -100
        elif max_speed > -1:
            max_speed = -1
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_SPEED_BACKWARD,
            0,
            max_speed, 0, 0, 0, 0, 0, 0
        )

    def set_pid(self, Kp, Ki, Kd):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_PID,
            0,
            Kp, Ki, Kd, 0, 0, 0, 0
        )

    def set_light(self, enable):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_LIGHT,
            0,
            enable, 0, 0, 0, 0, 0, 0
        )

    def set_camera(self, enable):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.SET_CAMERA,
            0,
            enable, 0, 0, 0, 0, 0, 0
        )

    def start_mag_calibration(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.START_MAG_CALIBRATION,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def get_status(self, status_id):
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            status_id.encode('utf-8'),
            -1
        )

    def get_all_status(self):
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )

    def receive_status(self):
        while True:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True)
            print(msg.param_id.decode('utf-8'), msg.param_value)
            status.update_status(msg.param_id.decode('utf-8'), msg.param_value)

MAV = MavlinkController()