import json
import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Autopilot.controller.utils.raspi_logger import LOG

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

_config_cache = {}
_last_update_time = 0
_cache_valid_time = 0.1
_cache_lock = threading.Lock()

def init_config():
    data = {
        "max_speed_forward": 100.0,
        "max_speed_backward": -100.0,
        "max_speed_dive": 100.0,
        "max_speed_surface": 100.0,
        "Kp_depth": 0,
        "Ki_depth": 0,
        "Kd_depth": 0,
        "Kp_autoheading": 0,
        "Ki_autoheading": 0,
        "Kd_autoheading": 0,
        "Kp_yaw": 0,
        "Ki_yaw": 0,
        "Kd_yaw": 0,
    }

    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        
        with open(CONFIG_PATH, "w") as file:
            json.dump(data, file, indent=4)
        
        with _cache_lock:
            global _config_cache, _last_update_time
            _config_cache = data.copy()
            _last_update_time = time.time()
            
        return data
    except Exception as e:
        LOG.error(f"Failed to initialize config file: {e}")
        return data

def update_config(key, value):
    try:
        if not os.path.exists(CONFIG_PATH):
            init_config()
        
        with _cache_lock:
            global _config_cache, _last_update_time
            if _config_cache:
                _config_cache[key] = value
        
        with open(CONFIG_PATH, "r") as file:
            data = json.load(file)
        
        data[key] = value
        
        with open(CONFIG_PATH, "w") as file:
            json.dump(data, file, indent=4)
            
        _last_update_time = time.time()
        return True
    except Exception as e:
        LOG.error(f"Failed to update config for key {key}: {e}")
        return False

def update_multiple(update_dict):
    if not update_dict:
        return False
        
    try:
        if not os.path.exists(CONFIG_PATH):
            init_config()
            
        with _cache_lock:
            global _config_cache, _last_update_time
            if _config_cache:
                for key, value in update_dict.items():
                    _config_cache[key] = value
        
        with open(CONFIG_PATH, "r") as file:
            data = json.load(file)
            
        for key, value in update_dict.items():
            data[key] = value
            
        with open(CONFIG_PATH, "w") as file:
            json.dump(data, file, indent=4)
            
        _last_update_time = time.time()
        return True
    except Exception as e:
        LOG.error(f"Failed to update multiple config: {e}")
        return False

def read_all_config():
    with _cache_lock:
        global _config_cache, _last_update_time
        current_time = time.time()
        if _config_cache and (current_time - _last_update_time) < _cache_valid_time:
            return _config_cache.copy()
    
    try:
        if not os.path.exists(CONFIG_PATH):
            return init_config()
            
        with open(CONFIG_PATH, "r") as file:
            data = json.load(file)
            
        with _cache_lock:
            _config_cache = data.copy()
            _last_update_time = time.time()
            
        return data
    except Exception as e:
        LOG.error(f"Failed to read config file: {e}")
        return False

def read_config(key, default=None):
    with _cache_lock:
        global _config_cache, _last_update_time
        current_time = time.time()
        if _config_cache and (current_time - _last_update_time) < _cache_valid_time:
            return _config_cache.get(key, default)
    
    try:
        data = read_all_config()
        return data.get(key, default)
    except Exception as e:
        LOG.error(f"Failed to read config for key {key}: {e}")
        return default

def read_multiple_config(keys):
    with _cache_lock:
        global _config_cache, _last_update_time
        current_time = time.time()
        if _config_cache and (current_time - _last_update_time) < _cache_valid_time:
            return {key: _config_cache.get(key) for key in keys}
        
    try:
        for key in keys:
            data = read_all_config()
            return {key: data.get(key) for key in keys}
    except Exception as e:
        LOG.error(f"Failed to read multiple config: {e}")
        return {key: None for key in keys}
    
def force_refresh():
    with _cache_lock:
        global _config_cache, _last_update_time
        _config_cache = {}
        _last_update_time = 0
    return read_all_config()

init_config()
# if __name__ == "__main__":
#     print(f"config file path: {CONFIG_PATH}")
    
#     init_config()
#     print("config initialized")
    
#     print("\nTesting config functions:")
#     update_config("depth", 5.5)
#     print(f"Updated depth: {read_config('depth')}")
    
#     update_multiple({"pitch": 10.2, "roll": -5.1})
#     print(f"Updated pitch: {read_config('pitch')}, roll: {read_config('roll')}")
    
#     all_config = read_all_config()
#     print("\nAll config values:")
#     for key, value in all_config.items():
#         print(f"  {key}: {value}")