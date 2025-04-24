import json
import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.utils.raspi_logger import LOG

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
        "internal_temp": 0,
        "external_temp": 0,
        "pitch": 0,
        "roll": 0,
        "heading": 0,
        "auto_heading": 0,
        "target_heading": 0,
        "auto_depth": 0,
        "target_depth": 0,
        "mode": "manual",
        "disconnected": 0,
        "light": 0,
        "camera": 0,
        "calibrated": 0,
        "position_x": 0,
        "position_y": 0,
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
        LOG.error(f"Failed to initialize status file: {e}")
        return data

def update_status(key, value):
    try:
        if not os.path.exists(STATUS_PATH):
            data = init_status()
        else:
            try:
                with open(STATUS_PATH, "r") as file:
                    data = json.load(file)
            except (json.JSONDecodeError, IOError) as e:
                LOG.error(f"Error reading status file: {e}")
                data = init_status()
        
        with _cache_lock:
            global _status_cache, _last_update_time
            if _status_cache is not None:
                _status_cache[key] = value
            else:
                _status_cache = data.copy()
                _status_cache[key] = value
            _last_update_time = time.time()
        
        data[key] = value
        
        with open(STATUS_PATH, "w") as file:
            json.dump(data, file, indent=4)
            
        return True
    except Exception as e:
        LOG.error(f"Failed to update status for key {key}: {e}")
        return False

def update_multiple(update_dict):
    if not update_dict:
        return False
        
    try:
        if not os.path.exists(STATUS_PATH):
            data = init_status()
        else:
            try:
                with open(STATUS_PATH, "r") as file:
                    data = json.load(file)
            except (json.JSONDecodeError, IOError) as e:
                LOG.error(f"Error reading status file: {e}")
                data = init_status()
            
        with _cache_lock:
            global _status_cache, _last_update_time
            if _status_cache is not None:
                for key, value in update_dict.items():
                    _status_cache[key] = value
            else:
                _status_cache = data.copy()
                for key, value in update_dict.items():
                    _status_cache[key] = value
            _last_update_time = time.time()
            
        for key, value in update_dict.items():
            data[key] = value
            
        with open(STATUS_PATH, "w") as file:
            json.dump(data, file, indent=4)
            
        return True
    except Exception as e:
        LOG.error(f"Failed to update multiple status: {e}")
        return False

def read_all_status():
    with _cache_lock:
        global _status_cache, _last_update_time
        current_time = time.time()
        if _status_cache and (current_time - _last_update_time) < _cache_valid_time:
            return _status_cache.copy()
    
    try:
        if not os.path.exists(STATUS_PATH):
            LOG.info("Status file not found, creating new one")
            return init_status()
            
        try:
            with open(STATUS_PATH, "r") as file:
                file_content = file.read().strip()
                if not file_content:
                    LOG.error("Empty status file detected")
                    return init_status()
                data = json.loads(file_content)
        except (json.JSONDecodeError, IOError) as e:
            LOG.error(f"JSON decode error in status file: {e}")
            LOG.info("Backing up corrupt status file and creating new one")
            # Backup corrupt file
            if os.path.exists(STATUS_PATH):
                backup_path = f"{STATUS_PATH}.backup.{int(time.time())}"
                try:
                    os.rename(STATUS_PATH, backup_path)
                    LOG.info(f"Backed up corrupt file to {backup_path}")
                except Exception as bak_err:
                    LOG.error(f"Failed to backup file: {bak_err}")
            return init_status()
            
        with _cache_lock:
            _status_cache = data.copy()
            _last_update_time = time.time()
            
        return data
    except Exception as e:
        LOG.error(f"Failed to read status file: {e}")
        return init_status()  # Return fresh status on any error

def read_status(key, default=0):
    with _cache_lock:
        global _status_cache, _last_update_time
        current_time = time.time()
        if _status_cache and (current_time - _last_update_time) < _cache_valid_time:
            return _status_cache.get(key, default)
    
    try:
        data = read_all_status()
        return data.get(key, default)
    except Exception as e:
        LOG.error(f"Failed to read status for key {key}: {e}")
        return default

def read_multiple_status(keys):
    with _cache_lock:
        global _status_cache, _last_update_time
        current_time = time.time()
        if _status_cache and (current_time - _last_update_time) < _cache_valid_time:
            return {key: _status_cache.get(key) for key in keys}
    
    try:
        data = read_all_status()
        if data:
            return {key: data.get(key) for key in keys}
        else:
            LOG.error("read_all_status() returned None or False")
            return {key: None for key in keys}
    except Exception as e:
        LOG.error(f"Failed to read multiple status: {e}")
        return {key: None for key in keys}
    
def force_refresh():
    with _cache_lock:
        global _status_cache, _last_update_time
        _status_cache = {}
        _last_update_time = 0
    return read_all_status()

init_status()