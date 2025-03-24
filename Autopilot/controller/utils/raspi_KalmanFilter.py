class KalmanFilter:
    """Bộ lọc Kalman cho việc ước lượng góc"""
    
    def __init__(self, q_angle=0.01, q_bias=0.03, r_measure=0.03):
        """Khởi tạo bộ lọc Kalman với các thông số cho trước"""
        # Các tham số nhiễu
        self.Q_angle = q_angle
        self.Q_bias = q_bias
        self.R_measure = r_measure
        
        # Trạng thái ước lượng
        self.angle = 0.0  # Góc ước lượng
        self.bias = 0.0   # Bias của Gyro
        
        # Ma trận hiệp phương sai
        self.P = [[0, 0], [0, 0]]
    
    def update(self, new_angle, new_rate, dt):
        """Cập nhật Kalman Filter với dữ liệu mới"""
        # Dự đoán (Predict)
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
        
        # Cập nhật ma trận P
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt
        
        # Tính Kalman Gain
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
        
        # Cập nhật với dữ liệu mới (Measurement Update)
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y
        
        # Điều chỉnh ma trận P
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp
        
        return self.angle