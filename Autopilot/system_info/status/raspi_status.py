import json
import Autopilot.system_info.sensor.raspi_sensor_read as sensor

def init_status():
    data ={
        "depth": 0,
        "temp": 0,
        "accel_x": 0,
        "accel_y": 0,
        "accel_z": 0,
        "gyro_x": 0,
        "gyro_y": 0,
        "gyro_z": 0,
        "mag_x": 0,
        "mag_y": 0,
        "mag_z": 0,
        "pitch": 0,
        "roll": 0,
        "heading": 0,
        "previous_temp": 0,
        "previous_accel_x": 0,
        "previous_accel_y": 0,
        "previous_accel_z": 0,
        "previous_gyro_x": 0,
        "previous_gyro_y": 0,
        "previous_gyro_z": 0,
        "previous_mag_x": 0,
        "previous_mag_y": 0,
        "previous_mag_z": 0,
        "previous_pitch": 0,
        "previous_roll": 0,
        "previous_heading": 0,
        "mag_calib_x": 0,
        "mag_calib_y": 0,
        "mag_calib_z": 0,
        "auto_heading": False,
        "auto_depth": False,
    }
    
    with open("status.json", "w") as file:
        json.dump(data, file)

def update_sensor_status():
    temp = sensor.read_temp_data()
    
    accel_x, accel_y, accel_z = sensor.read_accel_data_gravity_calibrated()
    
    gyro_x, gyro_y, gyro_z = sensor.read_gyro_data_dps()
    
    mag_x, mag_y, mag_z = sensor.read_mag_data_calibrated()
    
    pitch = sensor.read_angle_xz(accel_x, accel_z)
    
    roll = sensor.read_angle_yz(accel_y, accel_z)
    
    heading = sensor.read_angle_xy(mag_x, mag_y)

    try:
        with open("status.json", "r") as file:
            data = json.load(file)
    except FileNotFoundError:
        data = {}
    
    previous_accel_x = data.get("accel_x")
    previous_accel_y = data.get("accel_y")
    previous_accel_z = data.get("accel_z")
    previous_gyro_x = data.get("gyro_x")
    previous_gyro_y = data.get("gyro_y")
    previous_gyro_z = data.get("gyro_z")
    previous_mag_x = data.get("mag_x")
    previous_mag_y = data.get("mag_y")
    previous_mag_z = data.get("mag_z")
    previous_pitch = data.get("pitch")
    previous_roll = data.get("roll")
    previous_heading = data.get("heading")

    data.update({
        "depth": 0,
        "temp": temp,
        "accel_x": accel_x,
        "accel_y": accel_y,
        "accel_z": accel_z,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
        
        "pitch": pitch,
        "roll": roll,
        "heading": heading,

        "previous_accel_x": previous_accel_x,
        "previous_accel_y": previous_accel_y,
        "previous_accel_z": previous_accel_z,
        "previous_gyro_x": previous_gyro_x,
        "previous_gyro_y": previous_gyro_y,
        "previous_gyro_z": previous_gyro_z,
        "previous_mag_x": previous_mag_x,
        "previous_mag_y": previous_mag_y,
        "previous_mag_z": previous_mag_z,
        "previous_pitch": previous_pitch,
        "previous_roll": previous_roll,
        "previous_heading": previous_heading
    })

    with open("status.json", "w") as file:
        json.dump(data, file, indent=4)

def update_status(key, value):
    with open("status.json", "r") as file:
        data = json.load(file)
    data[key] = value
    with open("status.json", "w") as file:
        json.dump(data, file, indent=4)

def read_all_status():
    with open("status.json", "r") as file:
        data = json.load(file)
    return data

def read_status(key):
    with open("status.json", "r") as file:
        data = json.load(file)
    return data[key]

#init_status()