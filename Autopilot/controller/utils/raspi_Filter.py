import numpy as np
from collections import deque

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

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        self.sum = 0.0
        
    def update(self, new_value):
        if len(self.values) == self.window_size:
            self.sum -= self.values[0]
            
        self.values.append(new_value)
        self.sum += new_value
        
        return self.sum / len(self.values)
    
    def reset(self):
        self.values.clear()
        self.sum = 0.0

class CircularAverageFilter(MovingAverageFilter):
    def update(self, new_value):
        angle_rad = np.radians(new_value)
        x = np.cos(angle_rad)
        y = np.sin(angle_rad)
        
        if len(self.values) == self.window_size:
            old_value = self.values.popleft()
            old_rad = np.radians(old_value)
            self.sum_x -= np.cos(old_rad)
            self.sum_y -= np.sin(old_rad)
        
        self.values.append(new_value)
        
        if not hasattr(self, 'sum_x') or not hasattr(self, 'sum_y'):
            self.sum_x = 0
            self.sum_y = 0
            for val in self.values:
                val_rad = np.radians(val)
                self.sum_x += np.cos(val_rad)
                self.sum_y += np.sin(val_rad)
        else:
            self.sum_x += x
            self.sum_y += y
        
        avg_angle = np.degrees(np.arctan2(self.sum_y, self.sum_x))
        
        if avg_angle < 0:
            avg_angle += 360.0
            
        return avg_angle
    
    def reset(self):
        self.values.clear()
        self.sum_x = 0.0
        self.sum_y = 0.0

class MedianFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        
    def update(self, new_value):
        self.values.append(new_value)
        
        values_array = np.array(self.values)
        
        max_diff = np.max(values_array) - np.min(values_array)
        if max_diff > 180:
            values_array = np.where(values_array < 180, values_array + 360, values_array)
            median_value = np.median(values_array)
            if median_value >= 360:
                median_value -= 360
            return median_value
        else:
            return np.median(self.values)
    
    def reset(self):
        self.values.clear()

class AdaptiveFilter:
    def __init__(self, window_size=5, threshold=10.0):
        self.median_filter = MedianFilter(window_size)
        self.circular_avg_filter = CircularAverageFilter(window_size)
        self.threshold = threshold
        self.last_value = None
        
    def update(self, new_value):
        median_value = self.median_filter.update(new_value)
        
        avg_value = self.circular_avg_filter.update(median_value)
        
        if self.last_value is not None:
            diff = abs(avg_value - self.last_value)
            if diff > 180:
                diff = 360 - diff
                
            if diff > self.threshold:
                avg_value = self.last_value
        
        self.last_value = avg_value
        return avg_value
    
    def reset(self):
        self.median_filter.reset()
        self.circular_avg_filter.reset()
        self.last_value = None

def normalize_angle(angle):
    return angle % 360.0

def angle_difference(angle1, angle2):
    diff = abs(angle1 - angle2) % 360.0
    return min(diff, 360 - diff)