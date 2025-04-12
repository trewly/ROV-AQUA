import smbus2 as smbus
import time
import math
import threading
import numpy as np
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.utils.raspi_Filter import KalmanFilter, low_pass_filter
from Autopilot.utils.raspi_logger import LOG
from Autopilot.utils.raspi_Filter import MedianFilter, AdaptiveFilter, MovingAverageFilter


class MPU6050:
    G = 9.80665

    MPU6050_ADDR = 0x68
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

        self.prev_time = time.time()
        self.current_pitch = 0
        self.current_roll = 0

        self.pitch_median_filter = MedianFilter(window_size=5)
        self.roll_median_filter = MedianFilter(window_size=5)
        self.pitch_avg_filter = MovingAverageFilter(window_size=10)
        self.roll_avg_filter = MovingAverageFilter(window_size=10)
        
        self.pitch_adaptive_filter = AdaptiveFilter(window_size=7, threshold=5.0)
        self.roll_adaptive_filter = AdaptiveFilter(window_size=7, threshold=5.0)

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
            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x80)
            time.sleep(0.1)

            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)

            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x01)

            self.bus.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, 0x04)

            self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, self.GYRO_DLPF_41HZ)
            self.bus.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, self.GYRO_RANGE_250DPS)

            self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)
            self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG_2, self.ACCEL_DLPF_41HZ)

            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_2, 0x00)
            who_am_i = self.bus.read_byte_data(self.MPU6050_ADDR, self.WHO_AM_I)
            time.sleep(0.1)
            if who_am_i == 0x68 or who_am_i == 0x70:
                self.is_initialized = True
                
                self.current_velocity_x = 0.0
                self.current_velocity_y = 0.0
                self.current_velocity_z = 0.0
                self.velocity_last_update = time.time()
                self.velocity_reset_counter = 0
                
                self.stationary_counter = 0
                self.stationary_threshold = 5
                self.is_stationary = True
                self.velocity_confidence = 1.0
                
                if hasattr(self, 'accel_window'):
                    self.accel_window = []
                if hasattr(self, 'gyro_window'):
                    self.gyro_window = []
                    
                if hasattr(self, 'accel_x_filter'):
                    self.accel_x_filter.reset()
                    self.accel_y_filter.reset()
                    self.accel_z_filter.reset()
                    
                if hasattr(self, 'vx_filter'):
                    self.vx_filter.reset()
                    self.vy_filter.reset()
                    self.vz_filter.reset()
                
                self.kalman_pitch = KalmanFilter(q_angle=0.01, q_bias=0.003, r_measure=0.03)
                self.kalman_roll = KalmanFilter(q_angle=0.01, q_bias=0.003, r_measure=0.03)
                
                self.pitch_median_filter.reset()
                self.roll_median_filter.reset()
                self.pitch_avg_filter.reset()
                self.roll_avg_filter.reset()
                
                ax, ay, az = self.read_accel_data()
                pitch, roll = self.calculate_pitch_roll(ax, ay, az)

                self.kalman_pitch.angle = pitch
                self.kalman_roll.angle = roll
                
                self.current_pitch = pitch
                self.current_roll = roll
                self.prev_time = time.time()

                self.filtered_accel_x = 0.0
                self.filtered_accel_y = 0.0
                self.filtered_accel_z = 0.0

                return True
            else:
                LOG.error("MPU6050 not detected")
                self.is_initialized = False
                return False
        except Exception as e:
            LOG.error(f"Error initializing MPU6050: {e}")
            self.is_initialized = False
            return False

    def check_connection(self):
        try:
            who_am_i = self.bus.read_byte_data(self.MPU6050_ADDR, self.WHO_AM_I)
            if who_am_i == 0x68 or who_am_i == 0x70:
                self.is_initialized = True
                return True
            LOG.info("MPU6050 not detected")
            return False
        except Exception:
            LOG.error("Error checking connection")
            return False

    def read_data(self, register):
        try:
            if not self.is_initialized:
                LOG.error("MPU6050 not initialized, trying to initialize")
                if not self.initialize():
                    return 0

            high = self.bus.read_byte_data(self.MPU6050_ADDR, register)
            low = self.bus.read_byte_data(self.MPU6050_ADDR, register + 1)
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
            temp = self.bus.read_byte_data(self.MPU6050_ADDR, self.TEMP_OUT)
            temp = round((temp / 340) + 36.53, 2)
            status.update_status(key="internal_temp", value=temp)
            return temp
        except Exception:
            last_temp = status.read_status(key="internal_temp", default=25.0)
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
        try:
            ax, ay, az = self.read_accel_data()

            current_time = time.time()
            dt = current_time - self.velocity_last_update
            self.velocity_last_update = current_time

            ax_ms2 = ax * self.G
            ay_ms2 = ay * self.G
            az_ms2 = (az - 1.0) * self.G

            ACCEL_DEADBAND = 0.05
            ax_ms2 = 0.0 if abs(ax_ms2) < ACCEL_DEADBAND * self.G else ax_ms2
            ay_ms2 = 0.0 if abs(ay_ms2) < ACCEL_DEADBAND * self.G else ay_ms2
            az_ms2 = 0.0 if abs(az_ms2) < ACCEL_DEADBAND * self.G else az_ms2

            if not hasattr(self, 'accel_x_filter'):
                self.accel_x_filter = MovingAverageFilter(window_size=20)
                self.accel_y_filter = MovingAverageFilter(window_size=20)
                self.accel_z_filter = MovingAverageFilter(window_size=20)
                
            ax_filtered = self.accel_x_filter.update(ax_ms2)
            ay_filtered = self.accel_y_filter.update(ay_ms2)
            az_filtered = self.accel_z_filter.update(az_ms2)

            delta_vx = ax_filtered * dt
            delta_vy = ay_filtered * dt
            delta_vz = az_filtered * dt

            if abs(ax_filtered) < ACCEL_DEADBAND * self.G and abs(ay_filtered) < ACCEL_DEADBAND * self.G and abs(az_filtered) < ACCEL_DEADBAND * self.G:
                damping_factor = 0.95
                self.current_velocity_x *= damping_factor
                self.current_velocity_y *= damping_factor
                self.current_velocity_z *= damping_factor
            else:
                self.current_velocity_x += delta_vx
                self.current_velocity_y += delta_vy
                self.current_velocity_z += delta_vz

            self.current_velocity_x = low_pass_filter(
                self.current_velocity_x,
                self.current_velocity_x,
                self.VELOCITY_ALPHA
            )
            self.current_velocity_y = low_pass_filter(
                self.current_velocity_y,
                self.current_velocity_y,
                self.VELOCITY_ALPHA
            )
            self.current_velocity_z = low_pass_filter(
                self.current_velocity_z,
                self.current_velocity_z,
                self.VELOCITY_ALPHA
            )

            self.velocity_reset_counter += 1

            if self.velocity_reset_counter >= self.VELOCITY_RESET_THRESHOLD:
                self.velocity_reset_counter = 0
                self.current_velocity_x *= 0.25
                self.current_velocity_y *= 0.25
                self.current_velocity_z *= 0.25

            MAX_EXPECTED_VELOCITY = 5.0
            if (abs(self.current_velocity_x) > MAX_EXPECTED_VELOCITY or 
                abs(self.current_velocity_y) > MAX_EXPECTED_VELOCITY or 
                abs(self.current_velocity_z) > MAX_EXPECTED_VELOCITY):
                if (abs(ax_filtered) < 2 * ACCEL_DEADBAND * self.G and 
                    abs(ay_filtered) < 2 * ACCEL_DEADBAND * self.G and 
                    abs(az_filtered) < 2 * ACCEL_DEADBAND * self.G):
                    self.current_velocity_x *= 0.5
                    self.current_velocity_y *= 0.5
                    self.current_velocity_z *= 0.5

            horizontal_velocity = round(math.sqrt(self.current_velocity_x**2 + self.current_velocity_y**2), 2)
            vertical_velocity = round(self.current_velocity_z, 2)
            
            status.update_status(key="horizontal_velocity", value=horizontal_velocity)
            status.update_status(key="vertical_velocity", value=vertical_velocity)

            return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z

        except Exception as e:
            LOG.error(f"Error in get_velocity: {e}")
            return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z

    def get_velocity_enhanced(self):
        try:
            ax, ay, az = self.read_accel_data()
            
            if not hasattr(self, 'stationary_counter'):
                self.stationary_counter = 0
                self.stationary_threshold = 3
                self.is_stationary = False
                self.velocity_confidence = 1.0
                
            current_time = time.time()
            dt = current_time - self.velocity_last_update
            self.velocity_last_update = current_time
            
            dt = min(dt, 0.1)
            
            ax_ms2 = ax * self.G
            ay_ms2 = ay * self.G
            az_ms2 = (az - 1.0) * self.G
            
            ACCEL_DEADBAND = 0.02
            if abs(ax_ms2) < ACCEL_DEADBAND * self.G:
                ax_ms2 = 0.0
            if abs(ay_ms2) < ACCEL_DEADBAND * self.G:
                ay_ms2 = 0.0
            if abs(az_ms2) < ACCEL_DEADBAND * self.G:
                az_ms2 = 0.0
            
            if not hasattr(self, 'last_accel'):
                self.last_accel = (ax_ms2, ay_ms2, az_ms2)
            
            accel_change = (abs(ax_ms2 - self.last_accel[0]) + 
                            abs(ay_ms2 - self.last_accel[1]) + 
                            abs(az_ms2 - self.last_accel[2]))
            
            self.last_accel = (ax_ms2, ay_ms2, az_ms2)
            
            ACCEL_CHANGE_THRESHOLD = 0.1 * self.G
            if accel_change > ACCEL_CHANGE_THRESHOLD:
                self.is_stationary = False
                self.stationary_counter = 0
                
            is_currently_stationary = self.detect_stationary(window_size=10, threshold_accel=0.02, threshold_gyro=2.0)
            
            if is_currently_stationary:
                self.stationary_counter += 1
                if self.stationary_counter >= self.stationary_threshold:
                    if not self.is_stationary:
                        LOG.info("Device is now stationary - resetting velocity")
                        self.current_velocity_x = 0.0
                        self.current_velocity_y = 0.0
                        self.current_velocity_z = 0.0
                        self.velocity_confidence = 1.0
                    self.is_stationary = True
            else:
                self.stationary_counter = 0
                if self.is_stationary:
                    LOG.info("Device is now moving")
                self.is_stationary = False
                
            if not hasattr(self, 'accel_x_filter'):
                self.accel_x_filter = MovingAverageFilter(window_size=8)
                self.accel_y_filter = MovingAverageFilter(window_size=8)
                self.accel_z_filter = MovingAverageFilter(window_size=8)
                
                self.vx_filter = MovingAverageFilter(window_size=5) 
                self.vy_filter = MovingAverageFilter(window_size=5)
                self.vz_filter = MovingAverageFilter(window_size=5)
                
            ax_filtered = self.accel_x_filter.update(ax_ms2)
            ay_filtered = self.accel_y_filter.update(ay_ms2)
            az_filtered = self.accel_z_filter.update(az_ms2)
            
            if hasattr(self, 'prev_ax'):
                delta_vx = ((self.prev_ax + ax_filtered) / 2) * dt
                delta_vy = ((self.prev_ay + ay_filtered) / 2) * dt
                delta_vz = ((self.prev_az + az_filtered) / 2) * dt
            else:
                delta_vx = ax_filtered * dt
                delta_vy = ay_filtered * dt
                delta_vz = az_filtered * dt
                
            self.prev_ax = ax_filtered
            self.prev_ay = ay_filtered
            self.prev_az = az_filtered
            
            acceleration_weight = 1.0
            if not self.is_stationary and (abs(ax_filtered) > 0.1 * self.G or 
                                           abs(ay_filtered) > 0.1 * self.G or 
                                           abs(az_filtered) > 0.1 * self.G):
                acceleration_weight = 1.5
            
            if self.is_stationary:
                damping_factor = 0.7
                self.current_velocity_x *= damping_factor
                self.current_velocity_y *= damping_factor
                self.current_velocity_z *= damping_factor
            else:
                self.current_velocity_x += delta_vx * self.velocity_confidence * acceleration_weight
                self.current_velocity_y += delta_vy * self.velocity_confidence * acceleration_weight
                self.current_velocity_z += delta_vz * self.velocity_confidence * acceleration_weight
                
                self.velocity_confidence *= 0.998
                
            if not self.is_stationary:
                self.current_velocity_x = self.vx_filter.update(self.current_velocity_x)
                self.current_velocity_y = self.vy_filter.update(self.current_velocity_y)
                self.current_velocity_z = self.vz_filter.update(self.current_velocity_z)
            
            MAX_EXPECTED_VELOCITY = 5.0
            if abs(self.current_velocity_x) > MAX_EXPECTED_VELOCITY:
                self.current_velocity_x = np.sign(self.current_velocity_x) * MAX_EXPECTED_VELOCITY
                
            if abs(self.current_velocity_y) > MAX_EXPECTED_VELOCITY:
                self.current_velocity_y = np.sign(self.current_velocity_y) * MAX_EXPECTED_VELOCITY
                
            if abs(self.current_velocity_z) > MAX_EXPECTED_VELOCITY:
                self.current_velocity_z = np.sign(self.current_velocity_z) * MAX_EXPECTED_VELOCITY
            
            self.velocity_confidence = max(0.5, self.velocity_confidence)
            
            self.velocity_reset_counter += 1
            if self.velocity_reset_counter >= self.VELOCITY_RESET_THRESHOLD:
                self.velocity_reset_counter = 0
                confidence_factor = max(0.5, self.velocity_confidence ** 2)
                self.current_velocity_x *= confidence_factor
                self.current_velocity_y *= confidence_factor
                self.current_velocity_z *= confidence_factor
            
            horizontal_velocity = round(math.sqrt(self.current_velocity_x**2 + self.current_velocity_y**2), 2)
            vertical_velocity = round(self.current_velocity_z, 2)
            
            status.update_status(key="horizontal_velocity", value=horizontal_velocity)
            status.update_status(key="vertical_velocity", value=vertical_velocity)
            
            return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z
                
        except Exception as e:
            LOG.error(f"Error in get_velocity_enhanced: {e}")
            return self.current_velocity_x, self.current_velocity_y, self.current_velocity_z

    def get_orientation(self):
        try:
            ax, ay, az, gx, gy, gz = self.read_accel_gyro_data()

            current_time = time.time()
            dt = current_time - self.prev_time
            self.prev_time = current_time

            pitch_accel, roll_accel = self.calculate_pitch_roll(ax, ay, az)
            
            gyro_x_rad = math.radians(gx)
            gyro_y_rad = math.radians(gy)
            
            kalman_pitch = self.kalman_pitch.update(pitch_accel, gyro_y_rad, dt)
            kalman_roll = self.kalman_roll.update(roll_accel, gyro_x_rad, dt)
            
            pitch_median = self.pitch_median_filter.update(kalman_pitch)
            roll_median = self.roll_median_filter.update(kalman_roll)
            
            pitch_avg = self.pitch_avg_filter.update(pitch_median)
            roll_avg = self.roll_avg_filter.update(roll_median)
            
            self.current_pitch = round(pitch_avg, 2)
            self.current_roll = round(roll_avg, 2)
            
            status.update_status(key="pitch", value=self.current_pitch)
            status.update_status(key="roll", value=self.current_roll)

            return self.current_pitch, self.current_roll

        except Exception as e:
            LOG.error(f"Error in get_orientation: {e}")
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
                        "internal_temp": 0.0
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
                temp = status.read_status(key="internal_temp", default=25.0)

            sensor_data = {
                "pitch": round(pitch, 2),
                "roll": round(roll, 2),
                "velocity_x": round(vx, 2),
                "velocity_y": round(vy, 2),
                "velocity_z": round(vz, 2),
                "horizontal_velocity": round(horizontal_velocity, 2),
                "vertical_velocity": round(vz, 2),
                "internal_temp": round(temp, 2)
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
                "internal_temp": status.read_status(key="internal_temp", default=0.0)
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

            self.bus.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, gyro_range)

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

            self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, accel_range)

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

            self.bus.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, rate_divider)
            time.sleep(0.01)
            return True

        except Exception:
            LOG.error(f"Error setting sample rate: {rate_divider}")
            return False

    def set_filter_bandwidth(self, bandwidth):
        try:
            if not 0 <= bandwidth <= 7:
                return False

            self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, bandwidth)
            self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG_2, bandwidth)
            time.sleep(0.01)
            return True

        except Exception:
            LOG.error(f"Error setting filter bandwidth: {bandwidth}")
            return False

    def set_filter_bandwidth_gyro(self, bandwidth):
        try:
            if not 0 <= bandwidth <= 7:
                return False

            self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, bandwidth)
            time.sleep(0.01)
            return True

        except Exception:
            LOG.error(f"Error setting gyro filter bandwidth: {bandwidth}")
            return False

    def set_filter_bandwidth_accel(self, bandwidth):
        try:
            if not (0 <= bandwidth <= 6 or bandwidth == 8):
                return False

            self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG_2, bandwidth)
            time.sleep(0.01)
            return True

        except Exception:
            LOG.error(f"Error setting accel filter bandwidth: {bandwidth}")
            return False

    def set_power_mode(self, mode):
        try:
            current_mode = self.bus.read_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1) & 0xF8
            new_mode = current_mode | (mode & 0x07)
            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, new_mode)
            time.sleep(0.01)
            return True

        except Exception:
            LOG.error(f"Error setting power mode: {mode}")
            return False

    def detect_stationary(self, window_size=5, threshold_accel=0.02, threshold_gyro=2.0):
        ax, ay, az, gx, gy, gz = self.read_accel_gyro_data()
        
        if not hasattr(self, 'accel_window'):
            self.accel_window = []
            self.gyro_window = []
        
        self.accel_window.append((ax, ay, az))
        self.gyro_window.append((gx, gy, gz))
        
        if len(self.accel_window) > window_size:
            self.accel_window.pop(0)
        if len(self.gyro_window) > window_size:
            self.gyro_window.pop(0)
        
        min_samples = max(3, window_size // 3)
        if len(self.accel_window) < min_samples or len(self.gyro_window) < min_samples:
            return False
        
        ax_samples = np.array([a[0] for a in self.accel_window])
        ay_samples = np.array([a[1] for a in self.accel_window])
        az_samples = np.array([a[2] for a in self.accel_window])
        
        ax_var = np.var(ax_samples)
        ay_var = np.var(ay_samples)
        az_var = np.var(az_samples)
        
        gx_samples = np.array([g[0] for g in self.gyro_window])
        gy_samples = np.array([g[1] for g in self.gyro_window])
        gz_samples = np.array([g[2] for g in self.gyro_window])
        
        gx_max = np.max(np.abs(gx_samples))
        gy_max = np.max(np.abs(gy_samples))
        gz_max = np.max(np.abs(gz_samples))
        
        if gx_max > threshold_gyro * 1.5 or gy_max > threshold_gyro * 1.5 or gz_max > threshold_gyro * 1.5:
            return False
        
        accel_stable = (ax_var < threshold_accel*threshold_accel and 
                        ay_var < threshold_accel*threshold_accel and 
                        az_var < threshold_accel*threshold_accel)
        
        gyro_stable = (np.var(gx_samples) < threshold_gyro*threshold_gyro and 
                       np.var(gy_samples) < threshold_gyro*threshold_gyro and 
                       np.var(gz_samples) < threshold_gyro*threshold_gyro)
        
        return accel_stable and gyro_stable
    