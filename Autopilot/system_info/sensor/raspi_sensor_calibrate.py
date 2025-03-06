import raspi_sensor_read as sensor
import time

from Autopilot.system_info.status import raspi_status as status

def calibrate_mag():
    max_mag_x = 0
    max_mag_y = 0
    max_mag_z = 0
    min_mag_x = 999999
    min_mag_y = 999999
    min_mag_z = 999999
    print("Calibrating magnetometer...")
    for i in range(100):
        mag_x, mag_y, mag_z = sensor.read_mag_data()
        if mag_x > max_mag_x:
            max_mag_x = mag_x
        if mag_y > max_mag_y:
            max_mag_y = mag_y
        if mag_z > max_mag_z:
            max_mag_z = mag_z

        if mag_x < min_mag_x:
            min_mag_x = mag_x
        if mag_y < min_mag_y:
            min_mag_y = mag_y
        if mag_z < min_mag_z:
            min_mag_z = mag_z
        time.sleep(0.1)
    print("Calibration done")

    status.update_status(key="calib_mag_x", value=(max_mag_x + min_mag_x) / 2)
    status.update_status(key="calib_mag_y", value=(max_mag_y + min_mag_y) / 2)
    status.update_status(key="calib_mag_z", value=(max_mag_z + min_mag_z) / 2)