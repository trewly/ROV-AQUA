import smbus2 as smbus
import time
import math
import threading
import sys
import os
import numpy as np

from Autopilot.system_info.status import raspi_status as status
from Autopilot.controller.utils.raspi_Filter import KalmanFilter, low_pass_filter
from Autopilot.controller.utils.raspi_logger import LOG

class HMC5883L:
    HMC5883L_ADDR = 0x1E
    
    CONFIG_A = 0x00
    CONFIG_B = 0x01
    MODE = 0x02
    DATA_OUT_X_MSB = 0x03
    DATA_OUT_X_LSB = 0x04
    DATA_OUT_Z_MSB = 0x05
    DATA_OUT_Z_LSB = 0x06
    DATA_OUT_Y_MSB = 0x07
    DATA_OUT_Y_LSB = 0x08
    STATUS_REG = 0x09
    ID_REG_A = 0x0A
    ID_REG_B = 0x0B
    ID_REG_C = 0x0C
    
    SAMPLE_RATE_0_75HZ = 0x00
    SAMPLE_RATE_1_5HZ = 0x04
    SAMPLE_RATE_3HZ = 0x08
    SAMPLE_RATE_7_5HZ = 0x0C
    SAMPLE_RATE_15HZ = 0x10
    SAMPLE_RATE_30HZ = 0x14
    SAMPLE_RATE_75HZ = 0x18
    
    MEASUREMENT_NORMAL = 0x00
    MEASUREMENT_POS_BIAS = 0x01
    MEASUREMENT_NEG_BIAS = 0x02
    
    GAIN_1370 = 0x00
    GAIN_1090 = 0x20
    GAIN_820 = 0x40
    GAIN_660 = 0x60
    GAIN_440 = 0x80
    GAIN_390 = 0xA0
    GAIN_330 = 0xC0
    GAIN_230 = 0xE0
    
    MODE_CONTINUOUS = 0x00
    MODE_SINGLE = 0x01
    MODE_IDLE = 0x02
    MODE_SLEEP = 0x03
    
    STATUS_RDY = 0x01
    STATUS_LOCK = 0x02
    
    GAIN_SCALE = {
        GAIN_1370: 0.73,
        GAIN_1090: 0.92,
        GAIN_820: 1.22,
        GAIN_660: 1.52,
        GAIN_440: 2.27,
        GAIN_390: 2.56,
        GAIN_330: 3.03,
        GAIN_230: 4.35
    }
    
    DEFAULT_SAMPLE_RATE = SAMPLE_RATE_75HZ
    DEFAULT_MEASUREMENT_MODE = MEASUREMENT_NORMAL
    GAIN_HIGH = GAIN_1370
    DEFAULT_MODE = MODE_CONTINUOUS
    
    def __init__(self, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.is_initialized = False
        self.m_bias = np.array([0.0, 0.0, 0.0], dtype=float)
        self.m_scale = np.array([1.0, 1.0, 1.0], dtype=float)
        self.current_heading = 0.0
        self.declination = 0.0
        self.current_gain = self.GAIN_HIGH
        self.current_sample_rate = self.DEFAULT_SAMPLE_RATE
        self.update_thread = None
        self.running = False
        
    def initialize(self):
        try:
            config_a_value = self.DEFAULT_SAMPLE_RATE | self.DEFAULT_MEASUREMENT_MODE
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_A, config_a_value)
            
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_B, self.GAIN_HIGH)
            self.current_gain = self.GAIN_HIGH
            
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.MODE, self.DEFAULT_MODE)
            
            time.sleep(0.1)
            
            self.is_initialized = True
            return True

        except Exception:
            LOG.error("Error initializing HMC5883L")
            self.is_initialized = False
            return False
    
    def read_mag_status(self):
        try:
            self.bus.write_byte(self.HMC5883L_ADDR, self.STATUS_REG)
            return self.bus.read_byte(self.HMC5883L_ADDR)
        except Exception:
            LOG.error("Error reading magnetometer status")
            return 0    
        
    def set_gain(self, gain_setting):
        try:
            if gain_setting not in self.GAIN_SCALE:
                return False
                
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_B, gain_setting)
            self.current_gain = gain_setting
            time.sleep(0.01)
            
            return True
            
        except Exception:
            LOG.error(f"Error setting gain: {gain_setting}")
            return False
    
    def set_sample_rate(self, sample_rate):
        try:
            current_config = self.bus.read_byte_data(self.HMC5883L_ADDR, self.CONFIG_A)
            measurement_mode = current_config & 0x03
            
            config_a_value = sample_rate | measurement_mode
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_A, config_a_value)
            self.current_sample_rate = sample_rate
            return True
            
        except Exception:
            LOG.error(f"Error setting sample rate: {sample_rate}")
            return False
    
    def set_measurement_mode(self, mode):
        try:
            current_config = self.bus.read_byte_data(self.HMC5883L_ADDR, self.CONFIG_A)
            sample_rate = current_config & 0x1C
            
            config_a_value = sample_rate | mode
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_A, config_a_value)
            
            return True
            
        except Exception:
            LOG.error(f"Error setting measurement mode: {mode}")
            return False
    
    def set_operating_mode(self, mode):
        try:
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.MODE, mode)
            
            return True
            
        except Exception:
            LOG.error(f"Error setting operating mode: {mode}")
            return False
    
    def read_mag_data(self):
        try:
            if not self.is_initialized:
                if not self.initialize():
                    return np.array([0.0, 0.0, 0.0])
            if self.read_mag_status() & self.STATUS_RDY:
                self.bus.write_byte(self.HMC5883L_ADDR, self.DATA_OUT_X_MSB)
                
                data = []
                for _ in range(6):
                    data.append(self.bus.read_byte(self.HMC5883L_ADDR))
                
                x = (data[0] << 8) | data[1]
                z = (data[2] << 8) | data[3]
                y = (data[4] << 8) | data[5]
                
                x = x - 65536 if x > 32767 else x
                y = y - 65536 if y > 32767 else y
                z = z - 65536 if z > 32767 else z
            
            return np.array([x, y, z], dtype=float)

        except Exception:
            LOG.error("Error reading magnetometer data")
            if self.is_initialized:
                self.is_initialized = False
                try:
                    self.initialize()
                except:
                    pass
                    
            return np.array([0.0, 0.0, 0.0])
    
    def get_heading(self):
        mag = self.read_mag_data()
        if np.all(mag == 0):
            return self.current_heading
                
        mag_corrected = (mag - self.m_bias) * self.m_scale
            
        heading = math.atan2(mag_corrected[1], mag_corrected[0])
            
        heading += math.radians(self.declination)
            
        if heading < 0:
            heading += 2 * math.pi
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
            
        heading_degrees = math.degrees(heading)
            
        self.current_heading = heading_degrees
        status.update_status(key="heading", value=heading_degrees)
            
        return heading_degrees
            
    def calibrate(self, sample_count=1500):
        LOG.info("Starting calibration...")
        mag_min = np.array([999999, 999999, 999999], dtype=float)
        mag_max = np.array([-999999, -999999, -999999], dtype=float)
        
        start_time = time.time()
        samples_collected = 0
        
        while samples_collected < sample_count:
            try:
                mag = self.read_mag_data()
                if not np.all(mag == 0):
                    mag_min = np.minimum(mag_min, mag)
                    mag_max = np.maximum(mag_max, mag)
                    samples_collected += 1
                time.sleep(0.01)
                
            except Exception:
                time.sleep(0.1)
                
            if time.time() - start_time > 60:
                break
        
        LOG.info(f"Calibration completed")
        if samples_collected > 0:
            self.m_bias = (mag_max + mag_min) / 2
            scale_factors = (mag_max - mag_min) / 2
            avg_radius = np.mean(scale_factors)
            self.m_scale = avg_radius / scale_factors
            
            self.m_scale = np.nan_to_num(self.m_scale, nan=1.0)
            
            return True
        else:
            return False
    
    def update_loop(self, update_interval=0.05):
        self.running = True
        failure_count = 0
        
        while self.running:
            try:
                heading = self.get_heading()
                status.update_status(key="heading", value=heading)
                failure_count = 0
                time.sleep(update_interval)
            except Exception:
                failure_count += 1
                
                backoff_time = min(5, 0.1 * (2 ** min(failure_count, 5)))
                time.sleep(backoff_time)
                
                if failure_count % 10 == 0:
                    try:
                        self.initialize()
                    except:
                        pass
                        
    def start_update(self):
        if not self.is_initialized:
            if not self.initialize():
                return False
        
        if self.update_thread is None or not self.update_thread.is_alive():
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()
            return True
        return False
    
    def stop_update(self):
        self.running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            success = not self.update_thread.is_alive()
            return success
        return True
    

class MPU9250:
    G = 9.80665
    
    MPU9250_ADDR = 0x68
    PWR_MGMT_1 = 0x6B      
    PWR_MGMT_2 = 0x6C      
    SMPLRT_DIV = 0x19      
    CONFIG = 0x1A          
    GYRO_CONFIG = 0x1B     
    ACCEL_CONFIG = 0x1C    
    ACCEL_CONFIG_2 = 0x1D  
    INT_ENABLE = 0x38
    WHO_AM_I = 0x75
    
    TEMP_OUT = 0x41
    
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47
    
    ACCEL_SCALE = 16384.0  
    GYRO_SCALE = 131.0     
    
    GYRO_RANGE_250DPS = 0x00  
    GYRO_RANGE_500DPS = 0x08  
    GYRO_RANGE_1000DPS = 0x10 
    GYRO_RANGE_2000DPS = 0x18 
    
    ACCEL_RANGE_2G = 0x00    
    ACCEL_RANGE_4G = 0x08    
    ACCEL_RANGE_8G = 0x10    
    ACCEL_RANGE_16G = 0x18   
    
    ACCEL_DLPF_41HZ = 0x03   
    GYRO_DLPF_41HZ = 0x03
    
    ACCEL_ALPHA = 0.1  
    VELOCITY_ALPHA = 0.8  
    ACCEL_THRESHOLD = 0.05
    VELOCITY_RESET_THRESHOLD = 100

    GYRO_DLPF_250HZ = 0x00
    GYRO_DLPF_184HZ = 0x01
    GYRO_DLPF_92HZ = 0x02
    GYRO_DLPF_41HZ = 0x03
    GYRO_DLPF_20HZ = 0x04
    GYRO_DLPF_10HZ = 0x05
    GYRO_DLPF_5HZ = 0x06
    GYRO_DLPF_OFF = 0x07
    
    ACCEL_DLPF_218HZ = 0x00
    ACCEL_DLPF_99HZ = 0x02
    ACCEL_DLPF_45HZ = 0x03
    ACCEL_DLPF_21HZ = 0x04
    ACCEL_DLPF_10HZ = 0x05
    ACCEL_DLPF_5HZ = 0x06
    ACCEL_DLPF_OFF = 0x08
    
    def __init__(self, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.is_initialized = False
        
        self.kalman_pitch = KalmanFilter()
        self.kalman_roll = KalmanFilter()
        self.prev_time = time.time()
        self.current_pitch = 0
        self.current_roll = 0

        self.current_temp = 0.0

        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_velocity_z = 0.0
        self.velocity_last_update = time.time()
        self.velocity_reset_counter = 0
        
        self.filtered_accel_x = 0.0
        self.filtered_accel_y = 0.0
        self.filtered_accel_z = 0.0
        
        self.update_thread = None
        self.running = False
        
    def initialize(self):
        try:            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x01)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.SMPLRT_DIV, 0x04)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.CONFIG, self.GYRO_DLPF_41HZ)
            self.bus.write_byte_data(self.MPU9250_ADDR, self.GYRO_CONFIG, self.GYRO_RANGE_250DPS)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG_2, self.ACCEL_DLPF_41HZ)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_2, 0x00)
            
            who_am_i = self.bus.read_byte_data(self.MPU9250_ADDR, self.WHO_AM_I)
            if who_am_i == 0x71 or who_am_i == 0x73:  
                ax, ay, az = self.read_accel_data()
                pitch, roll = self.calculate_pitch_roll(ax, ay, az)
                
                self.kalman_pitch.angle = pitch
                self.kalman_roll.angle = roll
                
                self.current_pitch = pitch
                self.current_roll = roll
                
                self.filtered_accel_x = 0.0
                self.filtered_accel_y = 0.0
                self.filtered_accel_z = 0.0
                
                self.is_initialized = True
                return True
            else:
                LOG.error("MPU9250 not detected")
                self.is_initialized = False   
                return False
        except Exception:
            LOG.error("Error initializing MPU9250")
            self.is_initialized = False
            return False
            
    def check_connection(self):
        try:
            who_am_i = self.bus.read_byte_data(self.MPU9250_ADDR, self.WHO_AM_I)
            if who_am_i == 0x71 or who_am_i == 0x73:
                self.is_initialized = True
                return True
            LOG.info("MPU9250 not detected")
            return False
        except Exception:
            LOG.error("Error checking connection")
            return False
            
    def read_data(self, register):
        try:
            if not self.is_initialized:
                LOG.error("MPU9250 not initialized, trying to initialize")
                if not self.initialize():
                    return 0
                    
            high = self.bus.read_byte_data(self.MPU9250_ADDR, register)
            low = self.bus.read_byte_data(self.MPU9250_ADDR, register + 1)
            value = (high << 8) + low
            
            if value >= 32767:
                value -= 65536
                
            return value
        except Exception:
            if self.is_initialized:
                self.is_initialized = False
                try:
                    self.initialize()
                except:
                    pass
            return 0
            
    def read_temp_data(self):
        try:
            temp = self.bus.read_byte_data(self.MPU9250_ADDR, self.TEMP_OUT)
            temp = (temp / 340) + 36.53
            status.update_status(key="temp", value=temp)
            return temp
        except Exception:
            last_temp = status.read_status(key="temp", default=25.0)
            return last_temp
        
    def read_gyro_data(self):
        try:
            gyro_x = self.read_data(self.GYRO_XOUT_H) / self.GYRO_SCALE
            gyro_y = self.read_data(self.GYRO_YOUT_H) / self.GYRO_SCALE
            gyro_z = self.read_data(self.GYRO_ZOUT_H) / self.GYRO_SCALE
            return gyro_x, gyro_y, gyro_z
        except Exception:
            return 0.0, 0.0, 0.0
        
    def read_accel_data(self):
        try:
            accel_x = self.read_data(self.ACCEL_XOUT_H) / self.ACCEL_SCALE
            accel_y = self.read_data(self.ACCEL_YOUT_H) / self.ACCEL_SCALE
            accel_z = self.read_data(self.ACCEL_ZOUT_H) / self.ACCEL_SCALE
            return accel_x, accel_y, accel_z
        except Exception:
            return 0.0, 0.0, 1.0
        
    def read_accel_gyro_data(self):
        accel_x, accel_y, accel_z = self.read_accel_data()
        gyro_x, gyro_y, gyro_z = self.read_gyro_data()
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        
    def calculate_pitch_roll(self, ax, ay, az):
        try:
            pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)
            roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
            return pitch, roll
        except Exception:
            return 0, 0
            
    def get_velocity(self):
        ax, ay, az = self.read_accel_data()
                
        current_time = time.time()
        dt = current_time - self.velocity_last_update
        self.velocity_last_update = current_time
        
        ax_ms2 = ax * self.G
        ay_ms2 = ay * self.G
        az_ms2 = (az - 1.0) * self.G
        
        delta_vx = ax_ms2 * dt
        delta_vy = ay_ms2 * dt
        delta_vz = az_ms2 * dt
        
        self.current_velocity_x = low_pass_filter(
            self.current_velocity_x, 
            self.current_velocity_x + delta_vx, 
            self.VELOCITY_ALPHA
        )
        self.current_velocity_y = low_pass_filter(
            self.current_velocity_y, 
            self.current_velocity_y + delta_vy, 
            self.VELOCITY_ALPHA
        )
        self.current_velocity_z = low_pass_filter(
            self.current_velocity_z, 
            self.current_velocity_z + delta_vz, 
            self.VELOCITY_ALPHA
        )
        
        self.velocity_reset_counter += 1
        
        if self.velocity_reset_counter >= self.VELOCITY_RESET_THRESHOLD:
            self.velocity_reset_counter = 0
            self.current_velocity_x *= 0.5
            self.current_velocity_y *= 0.5
            self.current_velocity_z *= 0.5
        
        status.update_status(key="velocity_x", value=self.current_velocity_x)
        status.update_status(key="velocity_y", value=self.current_velocity_y)
        status.update_status(key="velocity_z", value=self.current_velocity_z)
        
        horizontal_velocity = math.sqrt(self.current_velocity_x**2 + self.current_velocity_y**2)
        status.update_status(key="horizontal_velocity", value=horizontal_velocity)
        status.update_status(key="vertical_velocity", value=self.current_velocity_z)
        
        return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z
            
            
    def get_orientation(self):
        try:
            ax, ay, az, gx, gy, gz = self.read_accel_gyro_data()
            
            current_time = time.time()
            dt = current_time - self.prev_time
            self.prev_time = current_time
            
            pitch_accel, roll_accel = self.calculate_pitch_roll(ax, ay, az)
            
            self.current_pitch = self.kalman_pitch.update(pitch_accel, gy, dt)
            self.current_roll = self.kalman_roll.update(roll_accel, gx, dt)
    
            status.update_status(key="pitch", value=self.current_pitch)
            status.update_status(key="roll", value=self.current_roll)
            
            return self.current_pitch, self.current_roll
            
        except Exception:
            return self.current_pitch, self.current_roll
            
    def read_all_sensors(self):
        try:
            if not self.is_initialized:
                if not self.initialize():
                    default_data = {
                        "pitch": 0.0,
                        "roll": 0.0,
                        "velocity_x": 0.0,
                        "velocity_y": 0.0,
                        "velocity_z": 0.0,
                        "horizontal_velocity": 0.0,
                        "vertical_velocity": 0.0,
                        "temp": 0.0
                    }
                    
                    for key, value in default_data.items():
                        status.update_status(key=key, value=value)
                        
                    return default_data
            
            try:
                pitch, roll = self.get_orientation()
            except Exception:
                pitch = status.read_status(key="pitch", default=0.0)
                roll = status.read_status(key="roll", default=0.0)
            
            try:
                vx, vy, vz = self.get_velocity()
                horizontal_velocity = math.sqrt(vx**2 + vy**2)
            except Exception:
                vx = status.read_status(key="velocity_x", default=0.0)
                vy = status.read_status(key="velocity_y", default=0.0)
                vz = status.read_status(key="velocity_z", default=0.0)
                horizontal_velocity = status.read_status(key="horizontal_velocity", default=0.0)
            
            try:
                temp = self.read_temp_data()
            except Exception:
                temp = status.read_status(key="temp", default=25.0)
            
            sensor_data = {
                "pitch": pitch,
                "roll": roll,
                "velocity_x": vx,
                "velocity_y": vy,
                "velocity_z": vz,
                "horizontal_velocity": horizontal_velocity,
                "vertical_velocity": vz,
                "temp": temp
            }
            status.update_multiple(sensor_data)
            return sensor_data
            
        except Exception:
            default_data = {
                "pitch": status.read_status(key="pitch", default=0.0),
                "roll": status.read_status(key="roll", default=0.0),
                "velocity_x": status.read_status(key="velocity_x", default=0.0),
                "velocity_y": status.read_status(key="velocity_y", default=0.0),
                "velocity_z": status.read_status(key="velocity_z", default=0.0),
                "horizontal_velocity": status.read_status(key="horizontal_velocity", default=0.0),
                "vertical_velocity": status.read_status(key="velocity_z", default=0.0),
                "temp": status.read_status(key="temp", default=0.0)
            }
            return default_data
    
    def update_loop(self, update_interval=0.01):
        self.running = True
        failure_count = 0
        
        while self.running:
            try:
                self.read_all_sensors()
                failure_count = 0
                time.sleep(update_interval)
            except Exception:
                failure_count += 1
                
                backoff_time = min(30, 0.1 * (2 ** min(failure_count, 8)))
                time.sleep(backoff_time)
                
                if failure_count % 10 == 0:
                    try:
                        self.initialize()
                    except:
                        pass
                                    
    def start_update(self):
        if not self.is_initialized:
            if not self.initialize():
                return False
        
        if self.update_thread is None or not self.update_thread.is_alive():
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()
            return True
        return False
        
    def stop_update(self):
        self.running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            success = not self.update_thread.is_alive()
            return success
        return True

    def set_gyro_range(self, gyro_range):
        try:
            if gyro_range not in [self.GYRO_RANGE_250DPS, self.GYRO_RANGE_500DPS, 
                                 self.GYRO_RANGE_1000DPS, self.GYRO_RANGE_2000DPS]:
                return False
                
            self.bus.write_byte_data(self.MPU9250_ADDR, self.GYRO_CONFIG, gyro_range)
            
            if gyro_range == self.GYRO_RANGE_250DPS:
                self.GYRO_SCALE = 131.0
            elif gyro_range == self.GYRO_RANGE_500DPS:
                self.GYRO_SCALE = 65.5
            elif gyro_range == self.GYRO_RANGE_1000DPS:
                self.GYRO_SCALE = 32.8
            elif gyro_range == self.GYRO_RANGE_2000DPS:
                self.GYRO_SCALE = 16.4
                
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting gyro range: {gyro_range}")
            return False
    
    def set_accel_range(self, accel_range):
        try:
            if accel_range not in [self.ACCEL_RANGE_2G, self.ACCEL_RANGE_4G, 
                                  self.ACCEL_RANGE_8G, self.ACCEL_RANGE_16G]:
                return False
                
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG, accel_range)
            
            if accel_range == self.ACCEL_RANGE_2G:
                self.ACCEL_SCALE = 16384.0
            elif accel_range == self.ACCEL_RANGE_4G:
                self.ACCEL_SCALE = 8192.0
            elif accel_range == self.ACCEL_RANGE_8G:
                self.ACCEL_SCALE = 4096.0
            elif accel_range == self.ACCEL_RANGE_16G:
                self.ACCEL_SCALE = 2048.0
                
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting accel range: {accel_range}")
            return False
    
    def set_sample_rate(self, rate_divider):
        try:
            if not 0 <= rate_divider <= 255:
                return False
                
            self.bus.write_byte_data(self.MPU9250_ADDR, self.SMPLRT_DIV, rate_divider)
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting sample rate: {rate_divider}")
            return False
    
    def set_filter_bandwidth(self, bandwidth):
        try:
            if not 0 <= bandwidth <= 7:
                return False
                
            self.bus.write_byte_data(self.MPU9250_ADDR, self.CONFIG, bandwidth)
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG_2, bandwidth)
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting filter bandwidth: {bandwidth}")
            return False

    def set_filter_bandwidth_gyro(self, bandwidth):
        try:
            if not 0 <= bandwidth <= 7:
                return False
                
            self.bus.write_byte_data(self.MPU9250_ADDR, self.CONFIG, bandwidth)
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting gyro filter bandwidth: {bandwidth}")
            return False
            
    def set_filter_bandwidth_accel(self, bandwidth):
        try:
            if not (0 <= bandwidth <= 6 or bandwidth == 8):
                return False
                
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG_2, bandwidth)
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting accel filter bandwidth: {bandwidth}")
            return False

    def set_power_mode(self, mode):
        try:
            current_mode = self.bus.read_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1) & 0xF8
            new_mode = current_mode | (mode & 0x07)
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, new_mode)
            time.sleep(0.01)
            return True
            
        except Exception:
            LOG.error(f"Error setting power mode: {mode}")
            return False

class SensorFusion:
    def __init__(self, mpu_instance, compass_instance):
        self.mpu = mpu_instance
        self.compass = compass_instance
        self.update_thread = None
        self.running = False
        self.update_interval = 0.013
        self.compass_update_counter = 0
    
    def update_loop(self):
        self.running = True
        failure_count = 0
        
        while self.running:
            try:
                self.mpu.read_all_sensors()
                self.compass.get_heading()                
                time.sleep(self.update_interval)
            except Exception:
                failure_count += 1
                
                backoff_time = min(5, 0.1 * (2 ** min(failure_count, 5)))
                time.sleep(backoff_time)
                
                if failure_count % 10 == 0:
                    try:
                        self.mpu.initialize()
                        self.compass.initialize()
                    except:
                        pass
    
    def start_update(self):
        mpu_init_success = self.mpu.initialize()
        compass_init_success = self.compass.initialize()
        
        if not (mpu_init_success or compass_init_success):
            return False
            
        if self.update_thread is None or not self.update_thread.is_alive():
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()
            return True
        return False
    
    def stop_update(self):
        self.running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            return not self.update_thread.is_alive()
        return True

mpu = MPU9250()
compass = HMC5883L()

sensor_fusion = SensorFusion(mpu, compass)

def initialize_sensors():
    return sensor_fusion.start_update()

def stop_sensors():
    return sensor_fusion.stop_update()

def read_all_sensors_with_heading():
    sensor_data = mpu.read_all_sensors()
    sensor_data["heading"] = compass.current_heading
    return sensor_data

if __name__ == "__main__":
    initialize_sensors()
    
    try:
        compass.calibrate()
    except KeyboardInterrupt:
        print("Calibration interrupted")
    
    print("Starting sensor reading loop. Press CTRL+C to exit.")
    try:
        for _ in range(100):
            sensor_data = read_all_sensors_with_heading()
            
            print(f"Pitch: {sensor_data['pitch']:.1f}°, Roll: {sensor_data['roll']:.1f}°")
            print(f"Heading: {sensor_data['heading']:.1f}°")
            print(f"Temp: {sensor_data['temp']:.1f}°C")
            print("-" * 30)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        stop_sensors()