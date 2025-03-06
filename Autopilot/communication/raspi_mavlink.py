from pymavlink import mavutil

from Autopilot.system_info.status import raspi_status as status
from Autopilot.system_info.sensor import raspi_sensor_calibrate as calibrate
from Autopilot.control.motor import raspi_motor_control as motor
from Autopilot.control.common import raspi_timer as timer

#master = mavutil.mavlink_connection("udp:0.0.0.0:50000")
#master.wait_hearbbeat() - threading

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

def received_cmd(master):
    msg = master.recv_match(blocking=True)
    if msg:
        return msg
    return None
            
def handle_cmd(master, cmd):
    if cmd == None:
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_WARNING,
            "No command received".encode('utf-8')
        )
        return
    
    master.mav.command_ack_send(
        cmd.command,
        mavutil.mavlink.MAV_RESULT_ACCEPT
    )

    if cmd.get_type() == "COMMAND_LONG":
        if status.read_status(key="mode") == "manual":
            if cmd.command == UP:
                timer.marked()
                motor.surface()
            elif cmd.command == DOWN:
                timer.marked()
                motor.dive()
            elif cmd.command == LEFT:
                timer.marked()
                motor.turn_left()
            elif cmd.command == RIGHT:
                timer.marked()
                motor.turn_right()
            elif cmd.command == FORWARD:
                timer.marked()
                motor.move_forward()
            elif cmd.command == BACKWARD:
                timer.marked()
                motor.move_backward()
            elif cmd.command == STOP:
                motor.stop_all()

        if timer.get_time_difference() > 1.5:
            motor.stop_all()  
        
        if cmd.command == SET_AUTO_HEADING:
            status.update_status(key="auto_heading", value=cmd.param1)
            status.update_status(key="target_heading", value=cmd.param2)
            status.update_status(key="mode", value="auto_heading")

        elif cmd.command == SET_AUTO_DEPTH:
            status.update_status(key="auto_depth", value=cmd.param1)
            status.update_status(key="target_depth", value=cmd.param2)
            status.update_status(key="mode", value="auto_depth")

        elif cmd.command == SET_PID:
            status.update_status(key="Kp", value=cmd.param1)
            status.update_status(key="Ki", value=cmd.param2)
            status.update_status(key="Kd", value=cmd.param3)

        elif cmd.command == SET_SPEED_FORWARD:
            status.update_status(key="max_speed_forward", value=cmd.param1)

        elif cmd.command == SET_SPEED_BACKWARD:
            status.update_status(key="max_speed_backward", value=cmd.param1)
        
        elif cmd.command == SET_LIGHT:
            status.update_status(key="light", value=cmd.param1)

        elif cmd.command == SET_CAMERA:
            status.update_status(key="camera", value=cmd.param1)
        
        elif cmd.command == START_MAG_CALIBRATION:
            calibrate.calibrate_mag()
        
    elif cmd.get_type() == "PARAM_REQUEST_READ":
        param_id = cmd.param_id.decode('utf-8').strip('\x00')
        param_value = status.read_status(key=param_id)
        master.mav.param_value_send(
            param_id.encode('utf-8'),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            -1,
            0
        )

    elif cmd.get_type() == "PARAM_REQUEST_LIST":
        for param_id, param_value in status.read_all_status().items():
            master.mav.param_value_send(
                param_id.encode('utf-8'),
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                -1,
                len(status.read_all_status())
            )