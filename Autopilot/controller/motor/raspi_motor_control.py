import pigpio
import sys
import os
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from Autopilot.system_info.status import raspi_status as status

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
                print("Failed to connect to pigpio daemon")
                return None
    return _pi

def scale_to_pwm(value):
    return 51 + ((value - (-100)) * (99 - 51) / 200)

class Motor:
    def __init__(self, pin, name=None):
        self.pi = get_pi_instance()
        if self.pi is None:
            raise ConnectionError("Could not connect to pigpio daemon")
            
        self.pin = pin
        self.name = name or f"Motor_PIN{pin}"
        self.last_speed = 0
        self.direction = STOP
        
        self.pi.set_PWM_frequency(self.pin, BASE_FREQUENCY)
        self.pi.set_PWM_range(self.pin, 100)
        self.pi.set_PWM_dutycycle(self.pin, DUTY_CYCLE_STOP)
        time.sleep(0.1)
        
    def get_direction(self):
        current_duty_cycle = self.pi.get_PWM_dutycycle(self.pin)
        if abs(current_duty_cycle - DUTY_CYCLE_STOP) < 0.5:
            return STOP
        elif current_duty_cycle > DUTY_CYCLE_STOP:
            return FORWARD
        else:
            return BACKWARD

    def set_dutycycle(self, duty_cycle):
        duty_cycle = min(DUTY_CYCLE_MAX_FORWARD, max(DUTY_CYCLE_MAX_BACKWARD, duty_cycle))
        self.pi.set_PWM_dutycycle(self.pin, duty_cycle)
        self.direction = self.get_direction()
        
    def set_speed(self, speed_percentage):
        speed_percentage = min(100, max(-100, speed_percentage))
        self.last_speed = speed_percentage
        dutycycle = scale_to_pwm(speed_percentage)
        self.set_dutycycle(dutycycle)
        return dutycycle
        
    def stop(self):
        if self.get_direction() != STOP:
            self.pi.set_PWM_dutycycle(self.pin, DUTY_CYCLE_STOP)
            self.direction = STOP
            self.last_speed = 0
            
    def set_speed_forward(self, status_key="max_speed_forward"):
        speed = status.read_status(status_key, 100)
        dutycycle = scale_to_pwm(speed)
        self.set_dutycycle(dutycycle)
        return speed
            
    def set_speed_backward(self, status_key="max_speed_backward"):
        speed = status.read_status(status_key, -100)
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
            print("All motors initialized successfully")
            return True
        else:
            print("Failed to initialize one or more motors")
            return False
            
    except Exception as e:
        print(f"Error initializing motors: {e}")
        return False

def self_balance(threshold=1.0):
    try:
        pitch = status.read_status("pitch", 0)
        roll = status.read_status("roll", 0)
        previous_pitch = status.read_status("previous_pitch", pitch)
        previous_roll = status.read_status("previous_roll", roll)
        
        pitch_diff = pitch - previous_pitch
        roll_diff = roll - previous_roll
        
        status.update_multiple({
            "previous_pitch": pitch,
            "previous_roll": roll
        })

        if abs(pitch_diff) > threshold:
            if pitch_diff > 0:
                LEFT_DEPTH_MOTOR.set_speed_forward()
                RIGHT_DEPTH_MOTOR.set_speed_forward()
            else:
                LEFT_DEPTH_MOTOR.set_speed_backward()
                RIGHT_DEPTH_MOTOR.set_speed_backward()

        if abs(roll_diff) > threshold:
            if roll_diff > 0:
                LEFT_DEPTH_MOTOR.set_speed_forward()
                RIGHT_DEPTH_MOTOR.set_speed_backward()
            else:
                LEFT_DEPTH_MOTOR.set_speed_backward()
                RIGHT_DEPTH_MOTOR.set_speed_forward()
                
        return True
    except Exception as e:
        print(f"Self-balance error: {e}")
        return False

def move_forward(speed=None):
    if speed is not None:
        status.update_status("max_speed_forward", speed)
    LEFT_MOTOR.set_speed_forward()
    RIGHT_MOTOR.set_speed_forward()

def move_backward(speed=None):
    if speed is not None:
        status.update_status("max_speed_backward", speed)
    LEFT_MOTOR.set_speed_backward()
    RIGHT_MOTOR.set_speed_backward()

def turn_left(speed=None):
    if speed is not None:
        status.update_multiple({
            "max_speed_backward": -speed,
            "max_speed_forward": speed
        })
    LEFT_MOTOR.set_speed_backward()
    RIGHT_MOTOR.set_speed_forward()

def turn_right(speed=None):
    if speed is not None:
        status.update_multiple({
            "max_speed_forward": speed,
            "max_speed_backward": -speed
        })
    LEFT_MOTOR.set_speed_forward()
    RIGHT_MOTOR.set_speed_backward()

def dive(speed=None):
    if speed is not None:
        status.update_status("max_speed_dive", speed)
    LEFT_DEPTH_MOTOR.set_speed_forward("max_speed_dive")
    RIGHT_DEPTH_MOTOR.set_speed_forward("max_speed_dive")

def surface(speed=None):
    if speed is not None:
        status.update_status("max_speed_surface", speed)
    LEFT_DEPTH_MOTOR.set_speed_backward("max_speed_surface")
    RIGHT_DEPTH_MOTOR.set_speed_backward("max_speed_surface")

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