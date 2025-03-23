from Autopilot.system_info.status import raspi_status as status
from Autopilot.control.motor import raspi_motor_control as rov
# from Autopilot.control.common.raspi_PID import PID

#pid = PID(1.0, 0.1, 0.05, setpoint=0)

def auto_heading(pid):
    if status.read_status(key="auto_heading"):
        heading = status.read_status(key="heading")
        target_heading = status.read_status(key="target_heading")

        pid.set_setpoint(setpoint=target_heading)
        pid.set_output_limits(status.read_status(key="max_speed_backward"), status.read_status("max_speed_forward"))
        
        output = pid.compute(input_value=heading)

        if abs(output) < 1:
            output = 0
        
        left_motor_speed = status.read_status(key="left_speed") + output
        right_motor_speed = status.read_status(key="right_speed") - output

        status.update_status(key="left_speed", value=left_motor_speed)
        status.update_status(key="right_speed", value=right_motor_speed)

        left_pwm = rov.scale_to_pwm(value=left_motor_speed)
        right_pwm = rov.scale_to_pwm(value=right_motor_speed)

        rov.set_auto_speed_forward(right_pwm=right_pwm, left_pwm=left_pwm)