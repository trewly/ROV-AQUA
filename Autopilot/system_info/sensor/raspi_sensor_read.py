import smbus2 as smbus
import time
import math
import threading
import sys
import os
import logging
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler

from Autopilot.system_info.status import raspi_status as status
from Autopilot.controller.utils.raspi_Filter import KalmanFilter, low_pass_filter

def setup_sensor_logger(name="MPU9250", log_subdir="../logs"):
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), log_subdir))
    os.makedirs(log_dir, exist_ok=True)
    
    current_date = datetime.now().strftime("%Y-%m-%d")
    log_file = os.path.join(log_dir, f"sensor_{current_date}.log")
    
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    
    if logger.handlers:
        logger.handlers.clear()
    
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    file_handler = TimedRotatingFileHandler(
        log_file, 
        when='midnight',
        interval=1,
        backupCount=30
    )
    file_handler.setLevel(logging.INFO)
    file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')  # Fixed 'levellevel' to 'levelname'
    file_handler.setFormatter(file_formatter)
    file_handler.suffix = "%Y-%m-%d"
    logger.addHandler(file_handler)
    
    logger.propagate = False
    
    return logger

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
    
    def __init__(self, bus_num=1):
        self.logger = setup_sensor_logger()
        self.logger.info("=" * 50)
        self.logger.info("MPU9250 sensor initialization started")
        self.logger.info("=" * 50)
        
        self.bus = smbus.SMBus(bus_num)
        self.is_initialized = False
        
        self.kalman_pitch = KalmanFilter()
        self.kalman_roll = KalmanFilter()
        self.prev_time = time.time()
        self.current_pitch = 0
        self.current_roll = 0
        
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
            self.logger.info("Initializing MPU9250 sensor...")
            
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
                self.logger.info(f"MPU9250/MPU9255 detected (0x{who_am_i:02X})")
                
                ax, ay, az = self.read_accel_data()
                pitch, roll = self.calculate_pitch_roll(ax, ay, az)
                
                self.kalman_pitch.angle = pitch
                self.kalman_roll.angle = roll
                
                self.current_pitch = pitch
                self.current_roll = roll
                
                self.filtered_accel_x = 0.0
                self.filtered_accel_y = 0.0
                self.filtered_accel_z = 0.0
                
                temp = self.read_temp_data()
                
                self.logger.info(f"Sensors initialized. Initial values: Pitch={pitch:.2f}°, Roll={roll:.2f}°, Temp={temp:.1f}°C")
                self.is_initialized = True
                return True
            else:
                self.logger.error(f"Unknown device ID: 0x{who_am_i:02X}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error initializing MPU9250: {e}")
            self.is_initialized = False
            return False
            
    def check_connection(self):
        try:
            who_am_i = self.bus.read_byte_data(self.MPU9250_ADDR, self.WHO_AM_I)
            if who_am_i == 0x71 or who_am_i == 0x73:
                return True
            return False
        except Exception as e:
            self.logger.warning(f"Connection check failed: {e}")
            return False
            
    def read_word(self, register):
        try:
            if not self.is_initialized:
                if not self.initialize():
                    return 0
                    
            high = self.bus.read_byte_data(self.MPU9250_ADDR, register)
            low = self.bus.read_byte_data(self.MPU9250_ADDR, register + 1)
            value = (high << 8) + low
            
            if value >= 0x8000:
                value -= 0x10000
                
            return value
        except Exception as e:
            if "I/O" in str(e) and self.is_initialized:
                self.logger.error("Sensor disconnected, trying to reinitialize...")
                self.is_initialized = False
                try:
                    self.initialize()
                except:
                    self.logger.error("Failed to reinitialize sensor")
            self.logger.error(f"Error reading from MPU: {e}")
            return 0
            
    def read_temp_data(self):
        try:
            temp = self.read_word(self.TEMP_OUT)
            temp = (temp / 340) + 36.53
            status.update_status(key="temp", value=temp)
            return temp
        except Exception as e:
            self.logger.error(f"Error reading temperature: {e}")
            last_temp = status.read_status(key="temp", default=25.0)
            return last_temp
        
    def read_gyro_data(self):
        try:
            gyro_x = self.read_word(self.GYRO_XOUT_H) / self.GYRO_SCALE
            gyro_y = self.read_word(self.GYRO_YOUT_H) / self.GYRO_SCALE
            gyro_z = self.read_word(self.GYRO_ZOUT_H) / self.GYRO_SCALE
            return gyro_x, gyro_y, gyro_z
        except Exception as e:
            self.logger.error(f"Error reading gyro data: {e}")
            return 0.0, 0.0, 0.0
        
    def read_accel_data(self):
        try:
            accel_x = self.read_word(self.ACCEL_XOUT_H) / self.ACCEL_SCALE
            accel_y = self.read_word(self.ACCEL_YOUT_H) / self.ACCEL_SCALE
            accel_z = self.read_word(self.ACCEL_ZOUT_H) / self.ACCEL_SCALE
            return accel_x, accel_y, accel_z
        except Exception as e:
            self.logger.error(f"Error reading accelerometer data: {e}")
            return 0.0, 0.0, 1.0
        
    def read_accel_gyro_data(self):
        accel_x, accel_y, accel_z = self.read_accel_data()
        gyro_x, gyro_y, gyro_z = self.read_gyro_data()
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        
    def filter_acceleration(self, ax, ay, az):
        self.filtered_accel_x = low_pass_filter(self.filtered_accel_x, ax, self.ACCEL_ALPHA)
        self.filtered_accel_y = low_pass_filter(self.filtered_accel_y, ay, self.ACCEL_ALPHA)
        self.filtered_accel_z = low_pass_filter(self.filtered_accel_z, az - 1.0, self.ACCEL_ALPHA)
        
        if abs(self.filtered_accel_x) < self.ACCEL_THRESHOLD:
            self.filtered_accel_x = 0.0
        if abs(self.filtered_accel_y) < self.ACCEL_THRESHOLD:
            self.filtered_accel_y = 0.0
        if abs(self.filtered_accel_z) < self.ACCEL_THRESHOLD:
            self.filtered_accel_z = 0.0
            
        return self.filtered_accel_x, self.filtered_accel_y, self.filtered_accel_z
        
    def calculate_pitch_roll(self, ax, ay, az):
        try:
            pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)
            roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
            return pitch, roll
        except Exception as e:
            self.logger.error(f"Error calculating pitch/roll: {e}")
            return 0, 0
            
    def calculate_velocity(self):
        try:
            ax, ay, az = self.read_accel_data()
            
            filtered_ax, filtered_ay, filtered_az = self.filter_acceleration(ax, ay, az)
            
            current_time = time.time()
            dt = current_time - self.velocity_last_update
            self.velocity_last_update = current_time
            
            ax_ms2 = filtered_ax * self.G
            ay_ms2 = filtered_ay * self.G
            az_ms2 = filtered_az * self.G
            
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
                self.logger.debug("Velocity reset applied to reduce drift")
            
            status.update_status(key="velocity_x", value=self.current_velocity_x)
            status.update_status(key="velocity_y", value=self.current_velocity_y)
            status.update_status(key="velocity_z", value=self.current_velocity_z)
            
            horizontal_velocity = math.sqrt(self.current_velocity_x**2 + self.current_velocity_y**2)
            status.update_status(key="horizontal_velocity", value=horizontal_velocity)
            status.update_status(key="vertical_velocity", value=self.current_velocity_z)
            
            return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z
        
        except Exception as e:
            self.logger.error(f"Error calculating velocity: {e}")
            return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z
            
    def get_velocity(self):
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
            
        except Exception as e:
            self.logger.error(f"Error in get_orientation: {e}")
            return self.current_pitch, self.current_roll
            
    def read_all_sensors(self):
        try:
            if not self.is_initialized:
                if not self.initialize():
                    self.logger.error("Could not initialize sensors, using fallback values")
                    default_data = {
                        "pitch": 0.0,
                        "roll": 0.0,
                        "velocity_x": 0.0,
                        "velocity_y": 0.0,
                        "velocity_z": 0.0,
                        "horizontal_velocity": 0.0,
                        "vertical_velocity": 0.0,
                        "temp": 25.0
                    }
                    
                    for key, value in default_data.items():
                        status.update_status(key=key, value=value)
                        
                    return default_data
            
            try:
                pitch, roll = self.get_orientation()
            except Exception as e:
                self.logger.error(f"Failed to get orientation: {e}")
                pitch = status.read_status(key="pitch", default=0.0)
                roll = status.read_status(key="roll", default=0.0)
            
            try:
                vx, vy, vz = self.calculate_velocity()
                horizontal_velocity = math.sqrt(vx**2 + vy**2)
            except Exception as e:
                self.logger.error(f"Failed to calculate velocity: {e}")
                vx = status.read_status(key="velocity_x", default=0.0)
                vy = status.read_status(key="velocity_y", default=0.0)
                vz = status.read_status(key="velocity_z", default=0.0)
                horizontal_velocity = status.read_status(key="horizontal_velocity", default=0.0)
            
            try:
                temp = self.read_temp_data()
            except Exception as e:
                self.logger.error(f"Failed to read temperature: {e}")
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
            
            for key, value in sensor_data.items():
                status.update_status(key=key, value=value)
            
            return sensor_data
            
        except Exception as e:
            self.logger.error(f"Critical error in read_all_sensors: {e}")
            default_data = {
                "pitch": status.read_status(key="pitch", default=0.0),
                "roll": status.read_status(key="roll", default=0.0),
                "velocity_x": status.read_status(key="velocity_x", default=0.0),
                "velocity_y": status.read_status(key="velocity_y", default=0.0),
                "velocity_z": status.read_status(key="velocity_z", default=0.0),
                "horizontal_velocity": status.read_status(key="horizontal_velocity", default=0.0),
                "vertical_velocity": status.read_status(key="velocity_z", default=0.0),
                "temp": status.read_status(key="temp", default=25.0)
            }
            return default_data
    
    def update_loop(self, update_interval=0.01):
        self.logger.info("Sensor update thread started")
        self.running = True
        failure_count = 0
        
        while self.running:
            try:
                self.read_all_sensors()
                failure_count = 0
                time.sleep(update_interval)
            except Exception as e:
                failure_count += 1
                self.logger.error(f"Error in sensor update loop: {e}")
                
                backoff_time = min(30, 0.1 * (2 ** min(failure_count, 8)))
                self.logger.warning(f"Backing off for {backoff_time:.2f} seconds before retry")
                time.sleep(backoff_time)
                
                if failure_count % 10 == 0:
                    self.logger.info("Attempting sensor recovery")
                    try:
                        self.initialize()
                    except Exception as recovery_e:
                        self.logger.error(f"Recovery failed: {recovery_e}")
                        
        self.logger.info("Sensor update thread terminated")
            
    def start_update(self):
        if not self.is_initialized:
            if not self.initialize():
                self.logger.error("Failed to initialize sensors, update thread not started")
                return False
        
        if self.update_thread is None or not self.update_thread.is_alive():
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()
            self.logger.info("Sensor update thread started")
            return True
        return False
        
    def stop_update(self):
        self.running = False
        self.logger.info("Stopping sensor update thread...")
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            success = not self.update_thread.is_alive()
            if success:
                self.logger.info("Sensor update thread stopped successfully")
            else:
                self.logger.warning("Sensor update thread did not terminate within timeout")
            return success
        return True
        
    def run_test(self, duration=10):
        self.logger.info(f"Starting sensor test for {duration} seconds...")
        
        if not self.is_initialized:
            self.initialize()
            
        start_time = time.time()
        
        while time.time() - start_time < duration:
            data = self.read_all_sensors()
            
            if data:
                self.logger.info(f"Pitch: {data['pitch']:.2f}°, Roll: {data['roll']:.2f}°, Temp: {data['temp']:.1f}°C")
                self.logger.info(f"Velocity: X={data['velocity_x']:.2f} m/s, Y={data['velocity_y']:.2f} m/s, Z={data['velocity_z']:.2f} m/s")
                self.logger.info(f"Horizontal Velocity: {data['horizontal_velocity']:.2f} m/s")
            
            time.sleep(0.1)
        
        self.logger.info("Sensor test completed.")

mpu = MPU9250()

def initialize_sensors():
    mpu.initialize()
    mpu.start_update()

def stop_sensors():
    mpu.stop_update()


if __name__ == "__main__":
    mpu.run_test()