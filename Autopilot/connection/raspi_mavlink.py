import time
import threading
import sys
import os
import signal
import logging
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler
from enum import IntEnum
from typing import List
from pymavlink import mavutil

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.system_info.sensor import raspi_sensor_calibrate as calibrate
from Autopilot.controller.motor import raspi_motor_control as rov
from Autopilot.controller.camera import raspi_camera as camera

def setup_logger(name="MAVLink", log_subdir="../logs"):
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), log_subdir))
    os.makedirs(log_dir, exist_ok=True)
    
    current_date = datetime.now().strftime("%Y-%m-%d-%H")
    log_file = os.path.join(log_dir, f"mavlink_raspi_{current_date}.log")
    
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    
    if logger.handlers:
        return
    
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
    
    logger.info("=" * 50)
    logger.info(f"{name} Logger initialized")
    logger.info(f"Log file: {log_file}")
    logger.info("=" * 50)
    
    return logger

logger = None

class MavCommand(IntEnum):
    SURFACE = 1000
    DIVE = 1001
    LEFT = 1002
    RIGHT = 1003
    FORWARD = 1004
    BACKWARD = 1005
    STOP = 1006
    ROLL_LEFT = 1007
    ROLL_RIGHT = 1008

    SET_MANUAL = 1100
    SET_AUTO_HEADING = 1101
    SET_AUTO_DEPTH = 1102
    SET_PID_YAW = 1103
    SET_PID_DEPTH = 1104
    SET_PID_AUTOHEADING = 1105
    
    SET_SPEED_FORWARD = 1106
    SET_SPEED_BACKWARD = 1107
    SET_SPEED_DIVE = 1108
    SET_SPEED_SURFACE = 1109

    SET_LIGHT = 1110
    SET_CAMERA = 1111

    START_MAG_CALIBRATION = 1200
    START_CAMERA_STREAM = 1201

class MavlinkController:
    def __init__(self, receive_port=5000, send_ip="169.254.54.121", send_port=5001):
        global logger
        
        if logger is None:
            logger = setup_logger("MAVLink", "../logs")
            
        logger.info("Initializing MAVLink controller")
            
        self.receive_port = receive_port
        self.send_ip = send_ip
        self.send_port = send_port
        
        self.status_send_interval = 0.01
        self.heartbeat_timeout = 10.0
        self.last_heartbeat = time.time()
        
        self.running = False
        self.threads: List[threading.Thread] = []
        self.connection_lost_reported = False
        
        self.receiver = None
        self.transmitter = None
        
        self.status_params = [
            "roll", "pitch", "heading", "temp", "depth", 
            "horizontal_velocity", "vertical_velocity"
        ]
        
        self._lock = threading.RLock()
    
    def _handle_manual_command(self, command: MavCommand) -> None:
        command_handlers = {
            MavCommand.SURFACE: rov.surface,
            MavCommand.DIVE: rov.dive,
            MavCommand.LEFT: rov.turn_left,
            MavCommand.RIGHT: rov.turn_right,
            MavCommand.FORWARD: rov.move_forward,
            MavCommand.BACKWARD: rov.move_backward,
            MavCommand.ROLL_LEFT: rov.roll_left,
            MavCommand.ROLL_RIGHT: rov.roll_right,
            MavCommand.STOP: rov.stop_all,
        }
        
        handler = command_handlers.get(command)
        if handler:
            handler()
        else:
            logger.warning(f"Unknown manual command: {command}")
    
    def _handle_mode_commands(self, msg) -> None:
        command = msg.command
        
        if command == MavCommand.SET_MANUAL:
            status.update_multiple({
                "mode": "manual",
                "auto_heading": False,
                "auto_depth": False
            })
            rov.stop_all()
            
        elif command == MavCommand.SET_AUTO_HEADING:
            heading = max(0, min(360, msg.param1))
            status.update_multiple({
                "auto_heading": True,
                "target_heading": heading,
                "mode": "auto_heading"
            })
            
        elif command == MavCommand.SET_AUTO_DEPTH:
            depth = max(0, min(10, msg.param1))
            status.update_multiple({
                "auto_depth": True,
                "target_depth": depth,
                "mode": "auto_depth"
            })
            
    
    def _handle_configuration_commands(self, msg) -> None:
        command = msg.command
        
        if command == MavCommand.SET_SPEED_FORWARD:
            status.update_status(key="max_speed_forward", value=msg.param1)
            
        elif command == MavCommand.SET_SPEED_BACKWARD:
            status.update_status(key="max_speed_backward", value=-msg.param1)
            
        elif command == MavCommand.SET_SPEED_DIVE:
            status.update_status(key="max_speed_dive", value=-msg.param1)
            
        elif command == MavCommand.SET_SPEED_SURFACE:
            status.update_status(key="max_speed_surface", value=msg.param1)

        elif command == MavCommand.SET_PID_YAW:
            status.update_multiple({
                "Kp_yaw": msg.param1,
                "Ki_yaw": msg.param2,
                "Kd_yaw": msg.param3
            })
        
        elif command == MavCommand.SET_PID_DEPTH:
            status.update_multiple({
                "Kp_depth": msg.param1,
                "Ki_depth": msg.param2,
                "Kd_depth": msg.param3
            })
        
        elif command == MavCommand.SET_PID_AUTOHEADING:
            status.update_multiple({
                "Kp_autoheading": msg.param1,
                "Ki_autoheading": msg.param2,
                "Kd_autoheading": msg.param3
            })
        
        elif command == MavCommand.SET_LIGHT:
            status.update_status(key="light", value=msg.param1)
           
        elif command == MavCommand.SET_CAMERA:
            status.update_status(key="camera", value=msg.param1)
        
        elif command == MavCommand.START_MAG_CALIBRATION:
            calibrate.calibrate_mag()
            status.update_status(key="calibrated", value=True)
            try:
                self.transmitter.mav.param_value_send(
                    "calibrated".encode("ascii"),
                    1.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
                    0,
                    1
                )
            except Exception as e:
                logger.error(f"Failed to send calibration status: {e}")

        elif command == MavCommand.START_CAMERA_STREAM:
            camera.start_stream()

    def handle_received_msg(self, msg) -> None:
        if msg is None:
            return

        if msg.get_type() == "COMMAND_LONG":
            try:
                command = MavCommand(msg.command)
                
                important_commands = {
                    MavCommand.SET_MANUAL, 
                    MavCommand.SET_AUTO_HEADING,
                    MavCommand.SET_AUTO_DEPTH,
                    MavCommand.SET_PID_YAW,
                    MavCommand.SET_PID_DEPTH,
                    MavCommand.SET_PID_AUTOHEADING,
                    MavCommand.SET_SPEED_FORWARD,
                    MavCommand.SET_SPEED_BACKWARD,
                    MavCommand.SET_SPEED_DIVE,
                    MavCommand.SET_SPEED_SURFACE,
                    MavCommand.SET_LIGHT,
                    MavCommand.SET_CAMERA,
                    MavCommand.START_MAG_CALIBRATION
                }
                
                if command in important_commands:
                    if command == MavCommand.SET_MANUAL:
                        logger.info(f"Mode changed: Manual control")
                    elif command == MavCommand.SET_AUTO_HEADING:
                        logger.info(f"Mode changed: Auto heading with target {msg.param1}°")
                    elif command == MavCommand.SET_AUTO_DEPTH:
                        logger.info(f"Mode changed: Auto depth with target {msg.param1}m")
                    elif command == MavCommand.SET_PID:
                        logger.info(f"PID parameters set: Kp={msg.param1:.2f}, Ki={msg.param2:.2f}, Kd={msg.param3:.2f}")
                    elif command == MavCommand.SET_SPEED_FORWARD:
                        logger.info(f"Forward speed set to {msg.param1}%")
                    elif command == MavCommand.SET_SPEED_BACKWARD:
                        logger.info(f"Backward speed set to {msg.param1}%") 
                    elif command == MavCommand.SET_SPEED_DIVE:
                        logger.info(f"Dive speed set to {msg.param1}%")
                    elif command == MavCommand.SET_SPEED_SURFACE:
                        logger.info(f"Surface speed set to {msg.param1}%")
                    elif command == MavCommand.SET_LIGHT:
                        logger.info(f"Light set to {msg.param1}%")
                    elif command == MavCommand.SET_CAMERA:
                        mode_str = "ON" if int(msg.param1) == 1 else "OFF"
                        logger.info(f"Camera set to {mode_str}")
                    elif command == MavCommand.START_MAG_CALIBRATION:
                        logger.info(f"Starting magnetometer calibration")
                
                current_mode = status.read_status(key="mode")

                if current_mode == "manual" and command in {
                    MavCommand.SURFACE, MavCommand.DIVE,
                    MavCommand.LEFT, MavCommand.RIGHT,
                    MavCommand.FORWARD, MavCommand.BACKWARD,
                    MavCommand.STOP
                }:
                    self._handle_manual_command(command)

                self._handle_mode_commands(msg)

                self._handle_configuration_commands(msg)

            except ValueError:
                logger.warning(f"Received unknown command ID: {msg.command}")
            except Exception as e:
                logger.error(f"Error handling command: {e}", exc_info=True)

    def _handle_connection_lost(self) -> None:
        if not self.connection_lost_reported:
            logger.warning("Connection lost with ground station, initiating surface maneuver")
            self.connection_lost_reported = True
            
        status.update_status(key="disconnect", value=True)
        
        current_depth = status.read_status(key="depth", default=0)
        if current_depth > 0:
            rov.surface()

    def receive_messages(self) -> None:
        logger.info(f"MAVLink receiver started on port {self.receive_port}")
        
        while self.running:
            try:
                msg = self.receiver.recv_match(blocking=True, timeout=1)
                if msg:
                    with self._lock:
                        status.update_status(key="disconnect", value=False)
                        self.connection_lost_reported = False
                        
                        if msg.get_type() == "HEARTBEAT":
                            self.last_heartbeat = time.time()
                        elif msg.get_type() == "COMMAND_LONG":
                            self.handle_received_msg(msg)
                else:
                    with self._lock:
                        if time.time() - self.last_heartbeat > self.heartbeat_timeout:
                            if not status.read_status(key="disconnect", default=False):
                                self._handle_connection_lost()
            except ConnectionError as e:
                logger.error(f"Connection error in receiver: {e}")
                time.sleep(1)
            except Exception as e:
                logger.error(f"Error in receiver thread: {e}", exc_info=True)
                time.sleep(1)

    def send_status_updates(self):
        logger.info(f"MAVLink transmitter sending to {self.send_ip}:{self.send_port}")
        
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        last_send_time = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                if current_time - last_send_time >= self.status_send_interval:
                    last_send_time = current_time
                    
                    status_data = status.read_all_status()
                    
                    for key in self.status_params:
                        try:
                            value = float(status_data.get(key, 0))
                            param_id = key.ljust(16, '\0')
                            
                            self.transmitter.mav.param_value_send(
                                param_id.encode("ascii"),
                                value,
                                param_type,
                                0,
                                len(self.status_params)
                            )
                        except (ValueError, TypeError) as e:
                            logger.warning(f"Invalid value for {key}: {e}")
                        except Exception as e:
                            logger.error(f"Error sending {key}: {e}")
                    
                    # self.transmitter.mav.heartbeat_send(
                    #     mavutil.mavlink.MAV_TYPE_SUBMARINE,
                    #     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    #     0,
                    #     0,
                    #     mavutil.mavlink.MAV_STATE_ACTIVE
                    # )
                
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Error in transmitter thread: {e}", exc_info=True)
                time.sleep(1)

    def start(self):
        with self._lock:
            if self.running:
                logger.warning("MAVLink controller already running")
                return False
                
            self.running = True
        
        try:
            self.receiver = mavutil.mavlink_connection(f"udpin:0.0.0.0:{self.receive_port}")
            receiver_thread = threading.Thread(
                target=self.receive_messages, 
                daemon=True,
                name="MAVLink-Receiver"
            )
            receiver_thread.start()
            self.threads.append(receiver_thread)
            
            self.transmitter = mavutil.mavlink_connection(f"udpout:{self.send_ip}:{self.send_port}")
            transmitter_thread = threading.Thread(
                target=self.send_status_updates, 
                daemon=True,
                name="MAVLink-Transmitter"
            )
            transmitter_thread.start()
            self.threads.append(transmitter_thread)
            
            logger.info("MAVLink controller started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start MAVLink controller: {e}", exc_info=True)
            self.stop()
            return False

    def stop(self):
        logger.info("Stopping MAVLink controller...")
        
        with self._lock:
            self.running = False
        
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=2)
                if thread.is_alive():
                    logger.warning(f"Thread {thread.name} did not terminate gracefully")
        
        self.threads = []
        
        if self.receiver:
            try:
                self.receiver.close()
            except Exception:
                pass
            self.receiver = None
            
        if self.transmitter:
            try:
                self.transmitter.close()
            except Exception:
                pass
            self.transmitter = None
            
        logger.info("MAVLink controller stopped")


controller = None

def init_mavlink(receive_port=5000, 
                 send_ip="169.254.54.121", 
                 send_port=5001):
    global controller, logger
    
    if logger is None:
        logger = setup_logger("MAVLink", "../logs")
    
    if controller is None:
        controller = MavlinkController(receive_port, send_ip, send_port)
    
    return controller.start()

def stop_mavlink():
    global controller
    
    if controller is not None:
        controller.stop()

def signal_handler(sig, frame) -> None:
    global logger
    
    if logger is None:
        logger = setup_logger("MAVLink", "../logs")
        
    logger.info("Received shutdown signal, stopping MAVLink connections...")
    stop_mavlink()
    sys.exit(0)

if __name__ == "__main__":
    logger = setup_logger("MAVLink", "../logs")
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if init_mavlink():
        logger.info("MAVLink communication started successfully")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Manual shutdown initiated...")
        finally:
            stop_mavlink()
    else:
        logger.error("Failed to initialize MAVLink communication")
        sys.exit(1)