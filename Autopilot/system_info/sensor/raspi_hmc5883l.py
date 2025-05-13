import smbus2 as smbus
import time
import math
import threading
import numpy as np
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.utils.raspi_logger import LOG
from Autopilot.utils.raspi_Filter import CircularAverageFilter, MedianFilter, AdaptiveFilter

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
            
            self.current_heading = round(heading_avg, 2)
            
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
    
hmc = HMC5883L()
while (not hmc.initialize()):
    time.sleep(1)
    print("Hello")

hmc.calibrate()

hmc.get_heading()