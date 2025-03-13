import pigpio
from Autopilot.system_info.status import raspi_status as status

FORWARD = 1
BACKWARD = -1
STOP = 0

DUTY_CYCLE_STOP = 75
DUTY_CYCLE_MAX_FORWARD = 99
DUTY_CYCLE_MAX_BACKWARD = 51

BASE_FREQUENCY = 500

def scale_to_pwm(value):
    new_value = 51 + ((value - (-100)) * (99 - 51) / (100 - (-100)))
    return new_value

class Motor:
    def __init__(self, pin):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            exit(0)
        self.pin = pin
        self.pi.set_PWM_frequency(self.pin, BASE_FREQUENCY)
        self.pi.set_PWM_range(self.pin, 100)
        self.pi.set_PWM_dutycycle(self.pin, DUTY_CYCLE_STOP)

    def get_direction(self):
        current_duty_cycle = self.pi.get_PWM_dutycycle(self.pin)
        if current_duty_cycle == DUTY_CYCLE_STOP:
            return STOP
        elif current_duty_cycle > DUTY_CYCLE_STOP:
            return FORWARD
        else:
            return BACKWARD

    def set_duty_cycle(self, duty_cycle):
        self.pi.set_PWM_dutycycle(self.pin, duty_cycle)

    def stop(self):
        if self.get_direction() == STOP:
            return
        else:
            self.pi.set_PWM_dutycycle(self.pin, DUTY_CYCLE_STOP)
    
    def increase_speed_forward(self):
        current_duty_cycle = self.pi.get_PWM_dutycycle(self.pin)
        while current_duty_cycle < min(scale_to_pwm(status.read_status(key="max_speed_forward")), DUTY_CYCLE_MAX_FORWARD) :
            self.pi.set_PWM_dutycycle(self.pin, current_duty_cycle + 1)
        
    def increase_speed_backward(self):
        current_duty_cycle = self.pi.get_PWM_dutycycle(self.pin)
        while current_duty_cycle > max(scale_to_pwm(status.read_status(key="max_speed_backward")), DUTY_CYCLE_MAX_BACKWARD):
            self.pi.set_PWM_dutycycle(self.pin, current_duty_cycle - 1)
    

LEFT_MOTOR = Motor(5)
RIGHT_MOTOR = Motor(6)
LEFT_DEPTH_MOTOR = Motor(7)
RIGHT_DEPTH_MOTOR = Motor(8)

def self_balance():
    pitch = status.read_status("pitch")
    roll = status.read_status("roll")
    previous_pitch = status.read_status("previous_pitch")
    previous_roll = status.read_status("previous_roll")

    if pitch - previous_pitch > 1:
        LEFT_DEPTH_MOTOR.increase_speed_forward()
        RIGHT_DEPTH_MOTOR.increase_speed_forward()
    elif pitch - previous_pitch < -1:
        LEFT_DEPTH_MOTOR.increase_speed_backward()
        RIGHT_DEPTH_MOTOR.increase_speed_backward()

    if roll - previous_roll > 1:
        LEFT_DEPTH_MOTOR.increase_speed_forward()
        RIGHT_DEPTH_MOTOR.increase_speed_backward()
    elif roll - previous_roll < -1:
        LEFT_DEPTH_MOTOR.increase_speed_backward()
        RIGHT_DEPTH_MOTOR.increase_speed_forward()

def move_forward():
    LEFT_MOTOR.increase_speed_forward()
    RIGHT_MOTOR.increase_speed_forward()

def move_backward():
    LEFT_MOTOR.increase_speed_backward()
    RIGHT_MOTOR.increase_speed_backward()

def turn_left():
    LEFT_MOTOR.increase_speed_backward()
    RIGHT_MOTOR.increase_speed_forward()

def turn_right():
    LEFT_MOTOR.increase_speed_forward()
    RIGHT_MOTOR.increase_speed_backward()

def dive():
    LEFT_DEPTH_MOTOR.increase_speed_forward()
    RIGHT_DEPTH_MOTOR.increase_speed_forward()

def surface():
    LEFT_DEPTH_MOTOR.increase_speed_backward()
    RIGHT_DEPTH_MOTOR.increase_speed_backward()

def stop(motor):
    motor.stop()

def stop_all():
    LEFT_MOTOR.stop()
    RIGHT_MOTOR.stop()
    LEFT_DEPTH_MOTOR.stop()
    RIGHT_DEPTH_MOTOR.stop()

def set_speed_forward(right_pwm, left_pwm):
    LEFT_MOTOR.set_duty_cycle(scale_to_pwm(left_pwm))
    RIGHT_MOTOR.set_duty_cycle(scale_to_pwm(right_pwm))

def set_speed_depth(right_pwm, left_pwm):
    LEFT_DEPTH_MOTOR.set_duty_cycle(scale_to_pwm(left_pwm))
    RIGHT_DEPTH_MOTOR.set_duty_cycle(scale_to_pwm(right_pwm))
