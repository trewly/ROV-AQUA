import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from connection import raspi_mavlink as mavlink
from controller.motor import raspi_motor_control as rov
from system_info.sensor import raspi_sensor_read as sensor
from system_info.status import raspi_status as status

def initilize_system():
    status.init_status()
    mavlink.initialize_mavlink()
    sensor.compass.initialize()
    
    time.sleep(0.5)
    sensor.compass.calibrate()
    while True:
        print(sensor.compass.get_heading())

    print("System initialized")