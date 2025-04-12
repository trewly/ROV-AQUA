import time
import struct
import atexit
import threading
import sys
import os
from smbus2 import SMBus

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.utils.raspi_logger import LOG
from Autopilot.utils.raspi_Filter import MovingAverageFilter

class BMP280:
    DEFAULT_I2C_ADDR = 0x76
    ALTERNATE_I2C_ADDR = 0x77
    
    REG_CHIPID = 0xD0
    REG_RESET = 0xE0
    REG_CTRL_MEAS = 0xF4
    REG_CONFIG = 0xF5
    REG_PRESS_MSB = 0xF7
    REG_TEMP_MSB = 0xFA
    REG_CALIB = 0x88
    
    MODE_SLEEP = 0x00
    MODE_FORCED = 0x01
    MODE_NORMAL = 0x03
    
    OVERSAMPLING_SKIP = 0x00
    OVERSAMPLING_X1 = 0x01
    OVERSAMPLING_X2 = 0x02
    OVERSAMPLING_X4 = 0x03
    OVERSAMPLING_X8 = 0x04
    OVERSAMPLING_X16 = 0x05
    
    FILTER_OFF = 0x00
    FILTER_X2 = 0x01
    FILTER_X4 = 0x02
    FILTER_X8 = 0x03
    FILTER_X16 = 0x04
    
    STANDBY_0_5MS = 0x00
    STANDBY_62_5MS = 0x01
    STANDBY_125MS = 0x02
    STANDBY_250MS = 0x03
    STANDBY_500MS = 0x04
    STANDBY_1000MS = 0x05
    STANDBY_2000MS = 0x06
    STANDBY_4000MS = 0x07
    
    DEFAULT_CHIP_ID = 0x58
    SEA_LEVEL_PRESSURE = 101325
    
    def __init__(self, bus_num=1, address=None):
        self.address = address or self.DEFAULT_I2C_ADDR
        self.bus = SMBus(bus_num)
        
        self.is_initialized = False
        self.t_fine = 0
        self.calib_data = None
        
        self.dig_T1 = 0
        self.dig_T2 = 0
        self.dig_T3 = 0
        self.dig_P1 = 0
        self.dig_P2 = 0
        self.dig_P3 = 0
        self.dig_P4 = 0
        self.dig_P5 = 0
        self.dig_P6 = 0
        self.dig_P7 = 0
        self.dig_P8 = 0
        self.dig_P9 = 0
        
        self.temperature = 0
        self.pressure = 0
        self.altitude = 0
        self.depth = 0
        
        self.temp_filter = MovingAverageFilter(window_size=10)
        self.pressure_filter = MovingAverageFilter(window_size=10)
        self.altitude_filter = MovingAverageFilter(window_size=10)
        self.depth_filter = MovingAverageFilter(window_size=10)
        
        self.update_thread = None
        self.running = False
        
        self.surface_pressure = self.SEA_LEVEL_PRESSURE
        
    def initialize(self):
        try:
            chip_id = self.bus.read_byte_data(self.address, self.REG_CHIPID)
            if chip_id != self.DEFAULT_CHIP_ID:
                LOG.error(f"Not a BMP280 sensor (ID = {hex(chip_id)})")
                return False
            
            self.bus.write_byte_data(self.address, self.REG_RESET, 0xB6)
            time.sleep(0.2)
            
            calib = self.bus.read_i2c_block_data(self.address, self.REG_CALIB, 24)
            calib_vals = struct.unpack_from('<HhhHhhhhhhhh', bytearray(calib))
            
            self.dig_T1, self.dig_T2, self.dig_T3 = calib_vals[0:3]
            self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9 = calib_vals[3:]
            
            ctrl_meas = (self.OVERSAMPLING_X1 << 5) | (self.OVERSAMPLING_X4 << 2) | self.MODE_NORMAL
            self.bus.write_byte_data(self.address, self.REG_CTRL_MEAS, ctrl_meas)
            
            config = (self.FILTER_X4 << 2) | (self.STANDBY_250MS << 5)
            self.bus.write_byte_data(self.address, self.REG_CONFIG, config)
            
            time.sleep(0.1)
            temp, press = self.read_temperature_pressure()
            self.temp_filter.reset(temp)
            self.pressure_filter.reset(press)
            
            self.is_initialized = True
            LOG.info("BMP280 initialized successfully")
            return True
            
        except Exception as e:
            LOG.error(f"Error initializing BMP280: {e}")
            self.is_initialized = False
            return False
    
    def read_raw_temp_pressure(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, self.REG_PRESS_MSB, 6)
            press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            return temp_raw, press_raw
        except Exception as e:
            LOG.error(f"Error reading raw data from BMP280: {e}")
            return 0, 0
    
    def compensate_temp(self, temp_raw):
        try:
            var1 = (((temp_raw >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
            var2 = (((((temp_raw >> 4) - self.dig_T1) * ((temp_raw >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
            self.t_fine = var1 + var2
            T = (self.t_fine * 5 + 128) >> 8
            return T / 100.0
        except Exception as e:
            LOG.error(f"Error compensating temperature: {e}")
            return 0
    
    def compensate_press(self, press_raw):
        try:
            var1 = self.t_fine - 128000
            var2 = var1 * var1 * self.dig_P6
            var2 = var2 + ((var1 * self.dig_P5) << 17)
            var2 = var2 + (self.dig_P4 << 35)
            var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
            var1 = (((1 << 47) + var1) * self.dig_P1) >> 33

            if var1 == 0:
                return 0

            p = 1048576 - press_raw
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            p = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
            return p / 256.0
        except Exception as e:
            LOG.error(f"Error compensating pressure: {e}")
            return 0
    
    def read_temperature_pressure(self):
        if not self.is_initialized:
            if not self.initialize():
                return 0, 0
                
        try:
            temp_raw, press_raw = self.read_raw_temp_pressure()
            temp = self.compensate_temp(temp_raw)
            press = self.compensate_press(press_raw)
            
            return round(temp, 2), round(press, 2)
        except Exception as e:
            LOG.error(f"Error reading temperature and pressure: {e}")
            return 0, 0
    
    def calculate_altitude(self, pressure, sea_level_pressure=None):
        if sea_level_pressure is None:
            sea_level_pressure = self.SEA_LEVEL_PRESSURE
            
        try:
            altitude = 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 1.0/5.255))
            return round(altitude, 2)
        except Exception as e:
            LOG.error(f"Error calculating altitude: {e}")
            return 0
    
    def calculate_depth(self, pressure):
        try:
            water_density = 1025
            gravity = 9.80665
            
            depth = (pressure - self.surface_pressure) / (water_density * gravity)
            return round(max(0, depth), 2)
        except Exception as e:
            LOG.error(f"Error calculating depth: {e}")
            return 0
    
    def set_surface_pressure(self):
        _, pressure = self.read_temperature_pressure()
        if pressure > 0:
            self.surface_pressure = pressure
            LOG.info(f"Surface water pressure set: {pressure} Pa")
            return True
        return False
    
    def update_loop(self, update_interval=0.1):
        self.running = True
        failure_count = 0
        
        while self.running:
            try:
                temp, press = self.read_temperature_pressure()
                
                filtered_temp = self.temp_filter.update(temp)
                filtered_press = self.pressure_filter.update(press)
                
                altitude = self.calculate_altitude(filtered_press)
                filtered_altitude = self.altitude_filter.update(altitude)
                
                depth = self.calculate_depth(filtered_press)
                filtered_depth = self.depth_filter.update(depth)
                
                self.temperature = round(filtered_temp, 2)
                self.pressure = round(filtered_press, 2)
                self.altitude = round(filtered_altitude, 2)
                self.depth = round(filtered_depth, 2)
                
                status.update_status(key="external_temp", value=self.temperature)
                status.update_status(key="depth", value=self.depth)
                
                failure_count = 0
                time.sleep(update_interval)
                
            except Exception as e:
                failure_count += 1
                LOG.error(f"Error in BMP280 update loop: {e}")
                
                backoff_time = min(5, 0.1 * (2 ** min(failure_count, 5)))
                time.sleep(backoff_time)
                
                if failure_count % 10 == 0:
                    try:
                        LOG.info("Attempting to reinitialize BMP280...")
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
            LOG.info("BMP280 update thread started")
            return True
        return False
    
    def stop_update(self):
        self.running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            success = not self.update_thread.is_alive()
            if success:
                LOG.info("BMP280 update thread stopped")
            return success
        return True