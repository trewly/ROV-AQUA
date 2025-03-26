import json
import os
import time
import threading

STATUS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "status.json")

_status_cache = {}
_last_update_time = 0
_cache_valid_time = 0.1
_cache_lock = threading.Lock()

def init_status():
    data = {
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
        "max_speed_backward": -100.0,
        "max_speed_dive": -100.0,
        "max_speed_surface": -100.0,
        "left_speed": 100,
        "right_speed": 100,
        "left_depth_speed": 100,
        "right_depth_speed": 100,
        "Kp": 0,
        "Ki": 0,
        "Kd": 0,
        "mode": "manual",
        "disconnect": False,
        "light": 0,
        "camera": 0,
        "velocity_x": 0,
        "velocity_y": 0,
        "velocity_z": 0
    }

    try:
        os.makedirs(os.path.dirname(STATUS_PATH), exist_ok=True)
        
        with open(STATUS_PATH, "w") as file:
            json.dump(data, file, indent=4)
        
        with _cache_lock:
            global _status_cache, _last_update_time
            _status_cache = data.copy()
            _last_update_time = time.time()
            
        return data
    except Exception as e:
        print(f"Error initializing status: {e}")
        return data

def update_status(key, value):
    try:
        if not os.path.exists(STATUS_PATH):
            init_status()
        
        with _cache_lock:
            global _status_cache, _last_update_time
            if _status_cache:
                _status_cache[key] = value
        
        # Sau đó cập nhật file
        with open(STATUS_PATH, "r") as file:
            data = json.load(file)
        
        data[key] = value
        
        with open(STATUS_PATH, "w") as file:
            json.dump(data, file, indent=4)
            
        _last_update_time = time.time()
        return True
    except Exception as e:
        print(f"Error updating status: {e}")
        return False

def update_multiple(update_dict):
    if not update_dict:
        return False
        
    try:
        if not os.path.exists(STATUS_PATH):
            init_status()
            
        with _cache_lock:
            global _status_cache, _last_update_time
            if _status_cache:
                for key, value in update_dict.items():
                    _status_cache[key] = value
        
        with open(STATUS_PATH, "r") as file:
            data = json.load(file)
            
        for key, value in update_dict.items():
            data[key] = value
            
        with open(STATUS_PATH, "w") as file:
            json.dump(data, file, indent=4)
            
        _last_update_time = time.time()
        return True
    except Exception as e:
        print(f"Error updating multiple status values: {e}")
        return False

def read_all_status():
    with _cache_lock:
        global _status_cache, _last_update_time
        current_time = time.time()
        if _status_cache and (current_time - _last_update_time) < _cache_valid_time:
            return _status_cache.copy()
    
    try:
        if not os.path.exists(STATUS_PATH):
            return init_status()
            
        with open(STATUS_PATH, "r") as file:
            data = json.load(file)
            
        with _cache_lock:
            _status_cache = data.copy()
            _last_update_time = time.time()
            
        return data
    except Exception as e:
        print(f"Error reading status: {e}")
        return init_status()

def read_status(key, default=None):
    with _cache_lock:
        global _status_cache, _last_update_time
        current_time = time.time()
        if _status_cache and (current_time - _last_update_time) < _cache_valid_time:
            return _status_cache.get(key, default)
    
    try:
        data = read_all_status()
        return data.get(key, default)
    except Exception as e:
        print(f"Error reading status key '{key}': {e}")
        return default

def force_refresh():
    with _cache_lock:
        global _status_cache, _last_update_time
        _status_cache = {}
        _last_update_time = 0
    return read_all_status()

if __name__ == "__main__":
    print(f"Status file path: {STATUS_PATH}")
    
    init_status()
    print("Status initialized")
    
    print("\nTesting status functions:")
    update_status("depth", 5.5)
    print(f"Updated depth: {read_status('depth')}")
    
    update_multiple({"pitch": 10.2, "roll": -5.1})
    print(f"Updated pitch: {read_status('pitch')}, roll: {read_status('roll')}")
    
    all_status = read_all_status()
    print("\nAll status values:")
    for key, value in all_status.items():
        print(f"  {key}: {value}")