import time
import threading
import atexit
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.utils.raspi_logger import LOG
from Autopilot.system_info.sensor.raspi_mpu6050 import MPU6050
from Autopilot.system_info.sensor.raspi_hmc5883l import HMC5883L
from Autopilot.system_info.sensor.raspi_bmp280 import BMP280

class SensorFusion:
    def __init__(self, mpu_instance, compass_instance, bmp_instance):
        self.mpu = mpu_instance
        self.compass = compass_instance
        self.bmp = bmp_instance
        self.update_thread = None
        self.running = False
        self.update_interval = 0.013
        self.compass_update_counter = 0
        self.bmp_update_counter = 0

    def initialize(self):
        LOG.info("Initializing all sensors")
        mpu_success = self.mpu.initialize()
        compass_success = self.compass.initialize()
        bmp_success = self.bmp.initialize()
        
        if mpu_success and compass_success and bmp_success:
            LOG.info("All sensors initialized successfully")
            return True
        else:
            LOG.warning(f"Sensor initialization status: MPU={mpu_success}, Compass={compass_success}, BMP280={bmp_success}")
            return False

    def update_loop(self):
        self.running = True
        failure_count = 0

        while self.running:
            try:
                # Update IMU sensors every cycle
                self.mpu.read_all_sensors()
                
                time.sleep(0.1)
                # Update compass every cycle
                self.compass.get_heading()
                
                # Update BMP280 every 5 cycles (slower rate)
                self.bmp_update_counter += 1
                if self.bmp_update_counter >= 5:
                    self.bmp_update_counter = 0
                    self.bmp.read_temperature_pressure()
                    
                time.sleep(self.update_interval)
            except Exception as e:
                failure_count += 1
                LOG.error(f"Error in sensor fusion update loop: {e}")

                backoff_time = min(5, 0.1 * (2 ** min(failure_count, 5)))
                time.sleep(backoff_time)

                if failure_count % 10 == 0:
                    try:
                        LOG.info("Attempting to reinitialize sensors...")
                        self.mpu.initialize()
                        self.compass.initialize()
                        self.bmp.initialize()
                    except Exception as init_err:
                        LOG.error(f"Failed to reinitialize sensors: {init_err}")

    def start_update(self):
        if self.update_thread is None or not self.update_thread.is_alive():
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()
            LOG.info("Sensor fusion update thread started")
            return True
        return False

    def stop_update(self):
        self.running = False

        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            success = not self.update_thread.is_alive()
            if success:
                LOG.info("Sensor fusion update thread stopped")
            return success
        return True

# Create sensor instances
mpu = MPU6050()
compass = HMC5883L()
bmp280 = BMP280()
sensor_fusion = SensorFusion(mpu, compass, bmp280)

def initialize_sensors():
    LOG.info("Starting sensor initialization")
    sensor_fusion.initialize()
    compass.calibrate()
    bmp280.set_surface_pressure()
    return sensor_fusion.start_update()

def stop_sensors():
    LOG.info("Stopping sensor fusion")
    return sensor_fusion.stop_update()

def cleanup():
    LOG.info("Cleaning up sensor resources")
    mpu.stop_update()
    compass.stop_update()
    bmp280.stop_update()
    sensor_fusion.stop_update()

atexit.register(cleanup)