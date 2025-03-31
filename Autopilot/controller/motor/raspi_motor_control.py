import pigpio
import sys
import os
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.controller.utils.raspi_logger import LOG

FORWARD = 1
BACKWARD = -1
STOP = 0

DUTY_CYCLE_STOP = 75
DUTY_CYCLE_MAX_FORWARD = 99
DUTY_CYCLE_MAX_BACKWARD = 51

BASE_FREQUENCY = 500

_pi = None
_pi_lock = threading.Lock()

def get_pi_instance():
    global _pi
    with _pi_lock:
        if _pi is None or not _pi.connected:
            _pi = pigpio.pi()
            if not _pi.connected:
                LOG.error("Failed to connect to pigpio daemon")
                return None
    return _pi

def scale_to_pwm(value):
    return 51 + ((value - (-100)) * (99 - 51) / 200)

class Motor:
    def __init__(self, pin, name=None):
        self.pi = get_pi_instance()
        if self.pi is None:
            LOG.error("Failed to initialize pigpio instance")
            
        self.pin = pin
        self.name = name or f"Motor_PIN{pin}"
        self.last_speed = 0
        self.direction = STOP
        
        self.pi.set_PWM_frequency(self.pin, BASE_FREQUENCY)
        self.pi.set_PWM_range(self.pin, 100)
        self.pi.set_PWM_dutycycle(self.pin, DUTY_CYCLE_STOP)
        time.sleep(0.1)
        
    def get_direction(self):
        try:
            current_duty_cycle = self.pi.get_PWM_dutycycle(self.pin)
            if abs(current_duty_cycle - DUTY_CYCLE_STOP) < 0.5:
                return STOP
            elif current_duty_cycle > DUTY_CYCLE_STOP:
                return FORWARD
            else:
                return BACKWARD
        except Exception as e:
            LOG.error(f"Error getting direction: {e}")
            return STOP

    def set_dutycycle(self, duty_cycle):
        try:
            duty_cycle = min(DUTY_CYCLE_MAX_FORWARD, max(DUTY_CYCLE_MAX_BACKWARD, duty_cycle))
            self.pi.set_PWM_dutycycle(self.pin, duty_cycle)
            self.direction = self.get_direction()

        except Exception as e:
            LOG.error(f"Error setting duty cycle: {e}")
        
    def set_speed(self, speed_percentage):
        try:
            speed_percentage = min(100, max(-100, speed_percentage))
            self.last_speed = speed_percentage
            dutycycle = scale_to_pwm(speed_percentage)
            self.set_dutycycle(dutycycle)
            return dutycycle
        
        except Exception as e:
            LOG.error(f"Error setting speed: {e}")
            return None
        
    def stop(self):
        try:
            if self.get_direction() != STOP:
                self.pi.set_PWM_dutycycle(self.pin, DUTY_CYCLE_STOP)
                self.direction = STOP
                self.last_speed = 0
        
        except Exception as e:
            LOG.error(f"Error stopping motor: {e}")
            
    def thrust_forward(self):
        speed = status.read_status("max_speed_forward", 100)
        dutycycle = scale_to_pwm(speed)
        self.set_dutycycle(dutycycle)
        return speed
            
    def thrust_backward(self):
        speed = status.read_status("max_speed_backward", -100)
        dutycycle = scale_to_pwm(speed)
        self.set_dutycycle(dutycycle)
        return speed
        
    def cleanup(self):
        self.stop()
        
    def __del__(self):
        try:
            self.stop()
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

def move_forward():
    LEFT_MOTOR.thrust_forward()
    RIGHT_MOTOR.thrust_forward()

def move_backward():
    LEFT_MOTOR.thrust_backward()
    RIGHT_MOTOR.thrust_backward()

def turn_left():
    LEFT_MOTOR.thrust_backward()
    RIGHT_MOTOR.thrust_forward()

def turn_right():
    LEFT_MOTOR.thrust_forward()
    RIGHT_MOTOR.thrust_backward()

def dive():
    LEFT_DEPTH_MOTOR.thrust_forward()
    RIGHT_DEPTH_MOTOR.thrust_forward()

def surface():
    LEFT_DEPTH_MOTOR.thrust_backward()
    RIGHT_DEPTH_MOTOR.thrust_backward()

def roll_right():
    LEFT_DEPTH_MOTOR.thrust_forward()
    RIGHT_DEPTH_MOTOR.thrust_backward()

def roll_left():
    LEFT_DEPTH_MOTOR.thrust_backward()
    RIGHT_DEPTH_MOTOR.thrust_forward()

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