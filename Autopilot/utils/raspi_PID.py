class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.output_limits = (None, None)

    def compute(self, input_value):
        error = self.setpoint - input_value
        
        self.integral += error

        derivative = error - self.previous_error
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.previous_error = error
        
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)
        
        return output

    def set_output_limits(self, min_output, max_output):
        self.output_limits = (min_output, max_output)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint