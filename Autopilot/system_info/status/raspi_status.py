import json

def init_status():
    data ={
        "depth": 0,
        "horizontal_velocity": 0,
        "vertical_velocity": 0,
        "temp": 0,
        "pitch": 0,
        "roll": 0,
        "heading": 0,
        "mag_calib_x": 0,
        "mag_calib_y": 0,
        "mag_calib_z": 0,
        "mag_calib_x_scale": 0,
        "mag_calib_y_scale": 0,
        "mag_calib_z_scale": 0,
        "auto_heading": False,
        "target_heading": 0,
        "auto_depth": False,
        "target_depth": 0,
        "max_speed_forward": 100.0,
        "max_speed_backward": -1.0,
        "max_speed_dive": 100.0,
        "max_speed_surface": 100.0,
        "left_speed": 100,
        "right_speed": 100,
        "left_depth_speed": 100,
        "right_depth_speed": 100,
        "Kp": 0,
        "Ki": 0,
        "Kd": 0,
        "mode": "manual",
        "disconnect": False
    }

    with open("/home/khanhisme1/Desktop/ROV-AQUA/Autopilot/system_info/status/status.json", "w") as file:
        json.dump(data, file)
    file.close()

def update_status(key, value):
    with open("/home/khanhisme1/Desktop/ROV-AQUA/Autopilot/system_info/status/status.json", "r") as file:
        data = json.load(file)
    data[key] = value
    with open("/home/khanhisme1/Desktop/ROV-AQUA/Autopilot/system_info/status/status.json", "w") as file:
        json.dump(data, file, indent=4)
    file.close()

def read_all_status():
    with open("/home/khanhisme1/Desktop/ROV-AQUA/Autopilot/system_info/status/status.json", "r") as file:
        data = json.load(file)
    file.close()
    return data

def read_status(key):
    with open("/home/khanhisme1/Desktop/ROV-AQUA/Autopilot/system_info/status/status.json", "r") as file:
        data = json.load(file)
    file.close()
    return data[key]

# init_status()