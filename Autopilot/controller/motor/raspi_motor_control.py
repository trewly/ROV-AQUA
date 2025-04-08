from gpiozero import PWMOutputDevice
import sys
import os
import time
import threading
import atexit

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.config import raspi_config as config
from Autopilot.controller.utils.raspi_logger import LOG
from Autopilot.connection.raspi_uart import UART

def check_emergency_stop():
    mode = status.read_status(key="mode", default="manual")
    return mode == "emergency" or mode == "emergency_shutdown" 

def move_forward():
    if not check_emergency_stop():
        UART.send_command("forward")
    else:
        LOG.warning("Emergency stop activated, cannot move forward")

def move_backward():
    if not check_emergency_stop():
        UART.send_command("backward")
    else:
        LOG.warning("Emergency stop activated, cannot move backward")

def turn_left():
    if not check_emergency_stop():
        UART.send_command("left")
    else:
        LOG.warning("Emergency stop activated, cannot turn left")

def turn_right():
    if not check_emergency_stop():
        UART.send_command("right")
    else:
        LOG.warning("Emergency stop activated, cannot turn right")

def dive():
    if not check_emergency_stop():
        UART.send_command("dive")
    else:
        LOG.warning("Emergency stop activated, cannot dive")

def surface():
    if not check_emergency_stop():
        UART.send_command("surface")
    else:
        LOG.warning("Emergency stop activated, cannot surface")

def stop_all():
    UART.send_command("stop_all")

def set_speed(motor, speed):
    if not check_emergency_stop():
        if motor == "left":
            UART.send_command("set_speed_left", speed)

        elif motor == "right":
            UART.send_command("set_speed_right", speed)

        elif motor == "left_depth":
            UART.send_command("set_speed_left_depth", speed)

        elif motor == "right_depth":
            UART.send_command("set_speed_right_depth", speed)
    else:
        LOG.warning("Emergency stop activated, cannot set speed")

def set_speed_forward(speed):
    if not check_emergency_stop():
        UART.send_command("set_speed_forward", speed)
    else:
        LOG.warning("Emergency stop activated, cannot set speed forward")

def set_speed_backward(speed):
    if not check_emergency_stop():
        UART.send_command("set_speed_backward", speed)
    else:
        LOG.warning("Emergency stop activated, cannot set speed backward")

def set_speed_surface(speed):
    if not check_emergency_stop():
        UART.send_command("set_speed_surface", speed)
    else:
        LOG.warning("Emergency stop activated, cannot set speed surface")

def set_speed_dive(speed):
    if not check_emergency_stop():
        UART.send_command("set_speed_dive", speed)
    else:
        LOG.warning("Emergency stop activated, cannot set speed dive")