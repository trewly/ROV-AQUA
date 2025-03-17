import json

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
    
    with open("Mission_planner/status/status.json", "w") as file:
        json.dump(data, file, indent=4)

def update_status(key, value):
    with open("Mission_planner/status/status.json", "r") as file:
        data = json.load(file)
    data[key] = value
    with open("Mission_planner/status/status.json", "w") as file:
        json.dump(data, file, indent=4)

def read_all_status():
    with open("Mission_planner/status/status.json", "r") as file:
        data = json.load(file)
    return data

def read_status(key):
    with open("Mission_planner/status/status.json", "r") as file:
        data = json.load(file)
    return data[key]

init_status()