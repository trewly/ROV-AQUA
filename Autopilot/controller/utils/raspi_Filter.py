class KalmanFilter:    
    def __init__(self, q_angle=0.01, q_bias=0.03, r_measure=0.03):
        self.Q_angle = q_angle
        self.Q_bias = q_bias
        self.R_measure = r_measure
        
        self.angle = 0.0
        self.bias = 0.0
        
        self.P = [[0, 0], [0, 0]]
    
    def update(self, new_angle, new_rate, dt):
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
        
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt
        
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
        
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y
        
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp
        
        return self.angle
    
def low_pass_filter(current_value, new_value, alpha):
    return current_value + alpha * (new_value - current_value)