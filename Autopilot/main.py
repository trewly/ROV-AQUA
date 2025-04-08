import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from connection.raspi_mavlink import MAV
from controller.motor import raspi_motor_control as rov
from system_info.sensor import raspi_sensor_read as sensor
from system_info.status import raspi_status as status

def initilize_system():
    status.init_status()
    MAV._initialize_connection()

    while True:
        print(sensor.compass.get_heading())

initilize_system()