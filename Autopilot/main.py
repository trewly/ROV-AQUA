import time
import signal
import threading
import sys
import os
import atexit
import logging
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler

from Autopilot.communication import raspi_mavlink as mav
from Autopilot.controller.motor import raspi_motor_control as rov
from Autopilot.system_info.status import raspi_status as status
from Autopilot.controller.camera import raspi_camera as camera
from Autopilot.system_info.sensor import raspi_sensor_read as sensor

def setup_system_logger(name="System", log_subdir="logs"):
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), log_subdir))
    os.makedirs(log_dir, exist_ok=True)
    
    current_date = datetime.now().strftime("%Y-%m-%d")
    log_file = os.path.join(log_dir, f"system_{current_date}.log")
    
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    
    if logger.handlers:
        logger.handlers.clear()
    
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    file_handler = TimedRotatingFileHandler(
        log_file, 
        when='midnight',
        interval=1,
        backupCount=30
    )
    file_handler.setLevel(logging.INFO)
    file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(file_formatter)
    file_handler.suffix = "%Y-%m-%d"
    logger.addHandler(file_handler)
    
    logger.propagate = False
    
    return logger

logger = setup_system_logger()

running = True
threads_status = {
    "sensors": False,
    "mavlink": False,
    "camera": False,
    "motors": False,
    "status": False
}

def monitor_threads():
    global threads_status
    
    while running:
        try:
            if threads_status["sensors"] and hasattr(sensor.mpu, "update_thread") and sensor.mpu.update_thread:
                if not sensor.mpu.update_thread.is_alive():
                    logger.warning("Sensor thread died, attempting to restart...")
                    if sensor.mpu.start_update():
                        logger.info("Sensor thread restarted successfully")
                    else:
                        logger.error("Failed to restart sensor thread")
            
            if threads_status["camera"]:
                is_active = False
                try:
                    is_active = camera.is_stream_active()
                except:
                    pass
                
                if not is_active:
                    logger.warning("Camera stream died, attempting to restart...")
                    try:
                        camera.start_stream()
                        logger.info("Camera stream restarted successfully")
                    except Exception as e:
                        logger.error(f"Failed to restart camera stream: {e}")
            
            time.sleep(5)
            
        except Exception as e:
            logger.error(f"Error in thread monitor: {e}")
            time.sleep(10)

def initialize_system():
    global threads_status
    
    logger.info("=" * 50)
    logger.info("SYSTEM INITIALIZATION STARTED")
    logger.info("=" * 50)
    
    logger.info("Initializing status system...")
    try:
        status.init_status()
        logger.info("Status system initialized")
        threads_status["status"] = True
    except Exception as e:
        logger.error(f"Failed to initialize status system: {e}")
        return False
    
    logger.info("Initializing sensors...")
    try:
        sensor_success = sensor.initialize_sensors()
        if sensor_success:
            logger.info("Sensors initialized successfully")
            if sensor.start_update():
                logger.info("Sensor update thread started")
                threads_status["sensors"] = True
            else:
                logger.warning("Failed to start sensor update thread")
        else:
            logger.warning("Sensor initialization failed, system will use default values")
    except Exception as e:
        logger.error(f"Error during sensor initialization: {e}")
    
    logger.info("Initializing motor controllers...")
    try:
        motor_success = rov.initialize_motors()
        if motor_success:
            logger.info("Motor controllers initialized successfully")
            threads_status["motors"] = True
            rov.stop_all()
        else:
            logger.error("Motor initialization failed! Motor control will not be available")
    except Exception as e:
        logger.error(f"Error during motor initialization: {e}")
    
    logger.info("Starting camera stream...")
    try:
        camera_success = camera.start_stream()
        if camera_success:
            logger.info("Camera stream started successfully")
            threads_status["camera"] = True
        else:
            logger.warning("Failed to start camera stream, video feed will not be available")
    except Exception as e:
        logger.error(f"Error starting camera stream: {e}")
    
    logger.info("Initializing MAVLink communication...")
    try:
        mavlink_success = mav.init_mavlink()
        if mavlink_success:
            logger.info("MAVLink communication initialized successfully")
            threads_status["mavlink"] = True
        else:
            logger.error("MAVLink communication failed to initialize, remote control will not be available")
    except Exception as e:
        logger.error(f"Error initializing MAVLink: {e}")
    
    monitor = threading.Thread(target=monitor_threads, daemon=True, name="ThreadMonitor")
    monitor.start()
    
    logger.info("=" * 50)
    logger.info("SYSTEM INITIALIZATION COMPLETE")
    logger.info("=" * 50)
    
    for system, running in threads_status.items():
        status_str = "RUNNING" if running else "FAILED"
        logger.info(f"Subsystem {system.upper()}: {status_str}")
    
    return threads_status["status"] and (threads_status["mavlink"] or threads_status["motors"])

def cleanup():
    logger.info("=" * 50)
    logger.info("SYSTEM SHUTTING DOWN")
    logger.info("=" * 50)
    
    logger.info("Stopping all motors...")
    try:
        if threads_status["motors"]:
            rov.stop_all()
            logger.info("All motors stopped")
    except Exception as e:
        logger.error(f"Error stopping motors: {e}")
    
    if threads_status["camera"]:
        logger.info("Stopping camera stream...")
        try:
            camera.stop_stream()
            logger.info("Camera stream stopped")
        except Exception as e:
            logger.error(f"Error stopping camera stream: {e}")
    
    if threads_status["sensors"]:
        logger.info("Stopping sensor updates...")
        try:
            sensor.mpu.stop_update()
            logger.info("Sensor updates stopped")
        except Exception as e:
            logger.error(f"Error stopping sensor updates: {e}")
    
    if threads_status["mavlink"]:
        logger.info("Stopping MAVLink communication...")
        try:
            mav.stop_mavlink()
            logger.info("MAVLink communication stopped")
        except Exception as e:
            logger.error(f"Error stopping MAVLink: {e}")
    
    try:
        if hasattr(rov, "cleanup"):
            rov.cleanup()
    except Exception as e:
        logger.error(f"Error during motor cleanup: {e}")
        
    logger.info("Cleanup complete, system shutting down")
    logger.info("=" * 50)

def signal_handler(sig, frame):
    global running
    logger.info(f"Received signal {sig}, initiating shutdown...")
    running = False
    cleanup()
    sys.exit(0)

atexit.register(cleanup)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

if __name__ == "__main__":
    try:
        initialization_successful = initialize_system()
        
        if not initialization_successful:
            logger.warning("Critical subsystems failed to initialize, system may not function correctly")
        
        logger.info("System is now running")
        
        last_status_print = 0
        while running:
            current_time = time.time()
            
            if current_time - last_status_print > 300:
                last_status_print = current_time
                logger.info("System running normally")    
            time.sleep(1)
            
    except Exception as e:
        logger.critical(f"Unhandled exception in main loop: {e}", exc_info=True)
    finally:
        running = False
        cleanup()
        sys.exit(1)