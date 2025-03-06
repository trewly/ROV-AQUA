import raspi_motor_control as motor
from Autopilot.system_info.status import raspi_status as status
# from . import raspi_PID as pid

#pid = pid.PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
#pid.set_output_limits(min_output=-100, max_output=100)

def auto_depth(pid):
    if status.read_status(key="auto_depth"):
        depth = status.read_status(key="depth")
        target_depth = status.read_status(key="target_depth")

        pid.set_setpoint(setpoint=target_depth)

        output = pid.compute(input_value=depth)

        if abs(output) < 1:
            output = 0
        
        left_depth_motor_speed = status.read_status(key="left_depth_speed") + output
        right_depth_motor_speed = status.read_status(key="right_depth_speed") + output

        status.update_status(key="left_depth_speed", value=left_depth_motor_speed)
        status.update_status(key="right_depth_speed", value=right_depth_motor_speed)

        left_pwm = motor.scale_to_pwm(value=left_depth_motor_speed)
        right_pwm = motor.scale_to_pwm(value=right_depth_motor_speed)

        motor.set_speed_depth(right_pwm=right_pwm, left_pwm=left_pwm)
