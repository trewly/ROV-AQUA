import smbus2 as smbus
import time
import math
import threading
import atexit
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.utils.raspi_Filter import KalmanFilter, low_pass_filter
from Autopilot.utils.raspi_logger import LOG
from Autopilot.utils.raspi_Filter import CircularAverageFilter, MedianFilter, AdaptiveFilter, MovingAverageFilter

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

    NUMBER_OF_SAMPLE_1 = 0x00
    NUMBER_OF_SAMPLE_2 = 0x20
    NUMBER_OF_SAMPLE_4 = 0x40
    NUMBER_OF_SAMPLE_8 = 0x60

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
        GAIN_1370: 0.88,
        GAIN_1090: 1.3,
        GAIN_820: 1.9,
        GAIN_660: 2.5,
        GAIN_440: 4.0,
        GAIN_390: 4.7,
        GAIN_330: 5.6,
        GAIN_230: 8.1
    }
    DEFAULT_NUMBER_OF_SAMPLES = NUMBER_OF_SAMPLE_8
    DEFAULT_SAMPLE_RATE = SAMPLE_RATE_75HZ
    DEFAULT_MEASUREMENT_MODE = MEASUREMENT_NORMAL
    GAIN_HIGH = GAIN_1370
    DEFAULT_MODE = MODE_CONTINUOUS

    def __init__(self, bus_num=1):
        status.update_status(key="calibrated", value=0)
        self.bus = smbus.SMBus(bus_num)
        self.is_initialized = False
        self.m_bias = np.array([0.0, 0.0, 0.0], dtype=float)
        self.m_scale = np.array([1.0, 1.0, 1.0], dtype=float)
        self.current_heading = 0.0
        self.declination = -0.0305
        self.current_gain = self.GAIN_HIGH
        self.current_sample_rate = self.DEFAULT_SAMPLE_RATE
        self.update_thread = None
        self.running = False
        
        self.heading_median_filter = MedianFilter(window_size=5)
        self.heading_avg_filter = CircularAverageFilter(window_size=10)
        self.heading_adaptive_filter = AdaptiveFilter(window_size=7, threshold=15.0)

    def initialize(self):
        try:
            config_a_value = self.DEFAULT_SAMPLE_RATE | self.DEFAULT_MEASUREMENT_MODE | self.DEFAULT_NUMBER_OF_SAMPLES
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_A, config_a_value)

            self.bus.write_byte_data(self.HMC5883L_ADDR, self.CONFIG_B, self.GAIN_HIGH)

            self.bus.write_byte_data(self.HMC5883L_ADDR, self.MODE, self.DEFAULT_MODE)
            self.current_gain = self.GAIN_HIGH

            time.sleep(0.5)

            self.is_initialized = True
            return True

        except Exception:
            LOG.error("Error initializing HMC5883L")
            self.is_initialized = False
            return False


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

    def read_mag_status(self):
        self.bus.write_byte(self.HMC5883L_ADDR, self.STATUS_REG)
        return self.bus.read_byte(self.HMC5883L_ADDR)

    def read_mag_data(self):
        self.bus.write_byte(self.HMC5883L_ADDR, self.DATA_OUT_X_MSB)

        data = self.bus.read_i2c_block_data(self.HMC5883L_ADDR, self.DATA_OUT_X_MSB, 6)

        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]

        x = x - 65536 if x > 32767 else x
        y = y - 65536 if y > 32767 else y
        z = z - 65536 if z > 32767 else z

        return np.array([x, y, z], dtype=float)


    def get_heading(self):
        mag_status = self.read_mag_status()
        if (mag_status & self.STATUS_RDY) and not (mag_status & self.STATUS_LOCK):
            mag = self.read_mag_data()

            mag_corrected = (mag - self.m_bias) * self.m_scale

            heading = math.atan2(mag_corrected[1], mag_corrected[0])

            heading += math.radians(self.declination)

            if heading < 0:
                heading += 2 * math.pi

            heading_degrees = math.degrees(heading)
            
            heading_median = self.heading_median_filter.update(heading_degrees)
            
            heading_avg = self.heading_avg_filter.update(heading_median)
            
            self.current_heading = heading_avg
            
            # heading_filtered = self.heading_adaptive_filter.update(heading_degrees)
            # self.current_heading = heading_filtered
            
            status.update_status(key="heading", value=self.current_heading)
            time.sleep(0.013)
        return self.current_heading

    def calibrate(self, sample_count=1500):
        LOG.info("Starting calibration...")
        mag_min = np.array([999998, 999998, 999998], dtype=float)
        mag_max = np.array([-999999, -999999, -999999], dtype=float)

        time.sleep(2)

        for i in range(sample_count):
            mag_status = self.read_mag_status()
            if mag_status & self.STATUS_RDY and not mag_status & self.STATUS_LOCK:
                mag = self.read_mag_data()
                mag_min = np.minimum(mag_min, mag)
                mag_max = np.maximum(mag_max, mag)
            time.sleep(0.013)

        LOG.info(f"Calibration completed")
        status.update_status(key="calibrated", value=1)
        self.m_bias = (mag_max + mag_min) / 2
        scale_factors = (mag_max - mag_min) / 2
        avg_radius = np.mean(scale_factors)
        self.m_scale = avg_radius / scale_factors

        self.m_scale = np.nan_to_num(self.m_scale, nan=1.0)
        time.sleep(1)
        return True

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
                LOG.error("MPU9250 not detected")
                self.is_initialized = False
                return False
        except Exception as e:
            LOG.error(f"Error initializing MPU9250: {e}")
            self.is_initialized = False
            return False

    def check_connection(self):
        try:
            who_am_i = self.bus.read_byte_data(self.MPU9250_ADDR, self.WHO_AM_I)
            if who_am_i == 0x68 or who_am_i == 0x70:
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

            horizontal_velocity = math.sqrt(self.current_velocity_x**2 + self.current_velocity_y**2)
            status.update_status(key="horizontal_velocity", value=horizontal_velocity)
            status.update_status(key="vertical_velocity", value=self.current_velocity_z)

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
            
            horizontal_velocity = math.sqrt(self.current_velocity_x**2 + self.current_velocity_y**2)
            status.update_status(key="horizontal_velocity", value=horizontal_velocity)
            status.update_status(key="vertical_velocity", value=self.current_velocity_z)
            
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
            
            self.current_pitch = pitch_avg
            self.current_roll = roll_avg
            
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

class SensorFusion:
    def __init__(self, mpu_instance, compass_instance):
        self.mpu = mpu_instance
        self.compass = compass_instance
        self.update_thread = None
        self.running = False
        self.update_interval = 0.013
        self.compass_update_counter = 0

    def initialize(self):
        self.mpu.initialize()
        self.compass.initialize()

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

# def read_all_sensors_with_heading():
#     sensor_data = mpu.read_all_sensors()
#     sensor_data["heading"] = compass.current_heading
#     return sensor_data

# if __name__ == "__main__":
#     compass.initialize()
#     time.sleep(0.5)
#     compass.calibrate()
#     while True:
#         print(compass.get_heading())

def cleanup():
    mpu.stop_update()
    compass.stop_update()
    sensor_fusion.stop_update()

atexit.register(cleanup)

if __name__ == "__main__":
    mpu.initialize()
    time.sleep(0.5)
    while True:
        mpu.get_velocity_enhanced()
        print(status.read_status(key="horizontal_velocity"))
