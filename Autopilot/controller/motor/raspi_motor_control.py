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

FORWARD = 1
BACKWARD = -1
STOP = 0

DUTY_CYCLE_STOP = 75 / 100
DUTY_CYCLE_MAX_FORWARD = 99 / 100
DUTY_CYCLE_MAX_BACKWARD = 51 / 100

BASE_FREQUENCY = 500

_pwm_instances = {}

def scale_to_pwm(value):
    duty =  51 + ((value - (-100)) * (99 - 51) / 200)
    return duty / 100

class Motor:
    def __init__(self, pin, name=None):
        self.pin = pin
        self.name = name or f"Motor_PIN{pin}"
        self.last_speed = 0
        self.direction = STOP

        self.pwm = PWMOutputDevice(pin, frequency=500, initial_value=DUTY_CYCLE_STOP)
        _pwm_instances[pin] = self.pwm

    def set_dutycycle(self, duty_cycle):
        self.pwm.value = duty_cycle

    def set_speed(self, speed_percentage):
        speed_percentage = min(100, max(-100, speed_percentage))
        self.last_speed = speed_percentage
        dutycycle = scale_to_pwm(speed_percentage)
        self.set_dutycycle(dutycycle)
        return dutycycle

    def stop(self):
        self.set_speed(0)

    def thrust_forward(self):
        speed = config.read_config("max_speed_forward", 100)
        dutycycle = scale_to_pwm(speed)
        self.set_dutycycle(dutycycle)
        return speed

    def thrust_backward(self):
        speed = config.read_config("max_speed_backward", -100)
        dutycycle = scale_to_pwm(speed)
        self.set_dutycycle(dutycycle)
        return speed

    def cleanup(self):
        self.stop()
        self.pwm.close()

    def __del__(self):
        try:
            self.cleanup()
        except:
            pass

motors = {}

def get_motor(pin, name=None):
    if pin not in motors:
        motors[pin] = Motor(pin, name)
    return motors[pin]

LEFT_MOTOR = None
RIGHT_MOTOR = None
LEFT_DEPTH_MOTOR = None
RIGHT_DEPTH_MOTOR = None

def initialize_motors(left_pin=5, right_pin=6, left_depth_pin=7, right_depth_pin=8):
    global LEFT_MOTOR, RIGHT_MOTOR, LEFT_DEPTH_MOTOR, RIGHT_DEPTH_MOTOR
    
    try:
        LEFT_MOTOR = get_motor(left_pin, "LEFT_MOTOR")
        RIGHT_MOTOR = get_motor(right_pin, "RIGHT_MOTOR")
        LEFT_DEPTH_MOTOR = get_motor(left_depth_pin, "LEFT_DEPTH_MOTOR")
        RIGHT_DEPTH_MOTOR = get_motor(right_depth_pin, "RIGHT_DEPTH_MOTOR")
        
        if all([LEFT_MOTOR, RIGHT_MOTOR, LEFT_DEPTH_MOTOR, RIGHT_DEPTH_MOTOR]):
            LOG.info("All motors initialized successfully")
            return True
        else:
            LOG.error("Failed to initialize one or more motors")
            return False
            
    except Exception as e:
        LOG.error(f"Error initializing motors: {e}")
        return False

def check_emergency_stop():
    mode = status.read_status(key="mode", default="manual")
    return mode == "emergency" or mode == "emergency_shutdown" 

def move_forward():
    if not check_emergency_stop():
        LEFT_MOTOR.thrust_forward()
        RIGHT_MOTOR.thrust_forward()
    else:
        LOG.warning("Emergency stop activated, cannot move forward")

def move_backward():
    if not check_emergency_stop():
        LEFT_MOTOR.thrust_backward()
        RIGHT_MOTOR.thrust_backward()
    else:
        LOG.warning("Emergency stop activated, cannot move backward")

def turn_left():
    if not check_emergency_stop():
        LEFT_MOTOR.thrust_backward()
        RIGHT_MOTOR.thrust_forward()
    else:
        LOG.warning("Emergency stop activated, cannot turn left")

def turn_right():
    if not check_emergency_stop():
        LEFT_MOTOR.thrust_forward()
        RIGHT_MOTOR.thrust_backward()
    else:
        LOG.warning("Emergency stop activated, cannot turn right")

def dive():
    if not check_emergency_stop():
        LEFT_DEPTH_MOTOR.thrust_backward()
        RIGHT_DEPTH_MOTOR.thrust_backward()
    else:
        LOG.warning("Emergency stop activated, cannot dive")

def surface():
    if not check_emergency_stop():
        LEFT_DEPTH_MOTOR.thrust_forward()
        RIGHT_DEPTH_MOTOR.thrust_forward()
    else:
        LOG.warning("Emergency stop activated, cannot surface")

def roll_right():
    if not check_emergency_stop():
        LEFT_DEPTH_MOTOR.thrust_forward()
        RIGHT_DEPTH_MOTOR.thrust_backward()
    else:
        LOG.warning("Emergency stop activated, cannot roll right")

def roll_left():
    if not check_emergency_stop():
        LEFT_DEPTH_MOTOR.thrust_backward()
        RIGHT_DEPTH_MOTOR.thrust_forward()
    else:
        LOG.warning("Emergency stop activated, cannot roll left")

def stop_all():
    for motor in motors.values():
        motor.stop()

def set_auto_speed_forward(right_speed, left_speed):
    RIGHT_MOTOR.set_speed(right_speed)
    LEFT_MOTOR.set_speed(left_speed)

def set_auto_speed_depth(right_speed, left_speed):
    RIGHT_DEPTH_MOTOR.set_speed(right_speed)
    LEFT_DEPTH_MOTOR.set_speed(left_speed)

def cleanup():
    stop_all()
    global _pi
    if _pi is not None and _pi.connected:
        _pi.stop()
        _pi = None

atexit.register(cleanup)