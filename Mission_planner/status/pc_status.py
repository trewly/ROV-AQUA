import json
import os
import time
import threading
import msvcrt  # Windows-specific file locking
import sys

STATUS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "status.json")

_status_cache = {}
_last_update_time = 0
_cache_valid_time = 0.1
_cache_lock = threading.Lock()
_file_lock = threading.Lock()  # Add file lock

def init_status():
    data = {
        "depth": 0,
        "horizontal_velocity": 0,
        "vertical_velocity": 0,
        "temp": 0,
        "pitch": 0,
        "roll": 0,
        "heading": 0,
        "auto_heading": 0,
        "target_heading": 0,
        "auto_depth": 0,
        "target_depth": 0,
        "mode": "manual",
        "disconnect": 1,
        "light": 0,
        "camera": 0,
        "calibrated": 0,
        "internal_temp": 0
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
    if value == 0:
        return True
        
    try:
        with _file_lock:
            if not os.path.exists(STATUS_PATH):
                init_status()
            
            # Update cache first
            with _cache_lock:
                global _status_cache, _last_update_time
                if _status_cache:
                    _status_cache[key] = value
            
            # Single file operation with proper error handling
            try:
                with open(STATUS_PATH, "r+") as file:
                    msvcrt.locking(file.fileno(), msvcrt.LK_NBLCK, 1)
                    try:
                        data = json.load(file)
                        data[key] = value
                        file.seek(0)
                        file.truncate()
                        json.dump(data, file, indent=4)
                    finally:
                        file.seek(0)
                        msvcrt.locking(file.fileno(), msvcrt.LK_UNLCK, 1)
                
                _last_update_time = time.time()
                return True
            except (IOError, OSError) as e:
                print(f"File operation error: {e}")
                return False
                
    except Exception as e:
        print(f"Error updating status: {e}")
        return False

def update_multiple(update_dict):
    if not update_dict:
        return False
        
    # Filter out values that are 0
    filtered_dict = {k: v for k, v in update_dict.items() if v != 0}
    
    if not filtered_dict:
        return True  # Return True if all values were 0
        
    try:
        if not os.path.exists(STATUS_PATH):
            init_status()
            
        with _cache_lock:
            global _status_cache, _last_update_time
            if _status_cache:
                for key, value in filtered_dict.items():
                    _status_cache[key] = value

        with _file_lock:
            try:
                with open(STATUS_PATH, "r+") as file:
                    msvcrt.locking(file.fileno(), msvcrt.LK_NBLCK, 1)
                    try:
                        data = json.load(file)
                        for key, value in filtered_dict.items():
                            data[key] = value
                            
                        file.seek(0)
                        file.truncate()
                        json.dump(data, file, indent=4)
                    finally:
                        file.seek(0)
                        msvcrt.locking(file.fileno(), msvcrt.LK_UNLCK, 1)
                
                _last_update_time = time.time()
                return True
            except (IOError, OSError) as e:
                print(f"File operation error: {e}")
                return False
                    
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
        return {}

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

init_status()

# while True:
#     update_multiple({"depth": 10, "horizontal_velocity": 5})
#     time.sleep(0.1)
#     update_multiple({"depth": 20, "horizontal_velocity": 10})
#     time.sleep(0.1)