import time
import threading
import sys
import os
import signal
import logging
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler
from enum import IntEnum
from typing import Dict, List, Optional, Any, Union
from pymavlink import mavutil

log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../logs"))
os.makedirs(log_dir, exist_ok=True)

current_date = datetime.now().strftime("%Y-%m-%d")
log_file = os.path.join(log_dir, f"mavlink_raspi_{current_date}.log")

logger = logging.getLogger("MAVLink")
logger.setLevel(logging.INFO)

if not logger.handlers:
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
logger.info("MAVLink Controller starting up")
logger.info(f"Log file: {log_file}")
logger.info("=" * 50)

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Autopilot.system_info.status import raspi_status as status
from Autopilot.system_info.sensor import raspi_sensor_calibrate as calibrate
from Autopilot.controller.motor import raspi_motor_control as rov

class MavCommand(IntEnum):
    SURFACE = 1000
    DIVE = 1001
    LEFT = 1002
    RIGHT = 1003
    FORWARD = 1004
    BACKWARD = 1005
    STOP = 1006

    SET_MANUAL = 1100
    SET_AUTO_HEADING = 1101
    SET_AUTO_DEPTH = 1102
    SET_PID = 1103
    
    SET_SPEED_FORWARD = 1104
    SET_SPEED_BACKWARD = 1105
    SET_SPEED_DIVE = 1107
    SET_SPEED_SURFACE = 1108

    SET_LIGHT = 1106
    SET_CAMERA = 1107

    START_MAG_CALIBRATION = 1200


class MavlinkController:

    def __init__(self, receive_port: int = 5000, 
                 send_ip: str = "169.254.54.121", 
                 send_port: int = 5001):
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
            
        elif command == MavCommand.SET_PID:
            status.update_multiple({
                "Kp": msg.param1,
                "Ki": msg.param2,
                "Kd": msg.param3
            })
    
    def _handle_configuration_commands(self, msg) -> None:
        command = msg.command
        
        if command == MavCommand.SET_SPEED_FORWARD:
            status.update_status(key="max_speed_forward", value=msg.param1)
            
        elif command == MavCommand.SET_SPEED_BACKWARD:
            status.update_status(key="max_speed_backward", value=msg.param1)
            
        elif command == MavCommand.SET_SPEED_DIVE:
            status.update_status(key="max_speed_dive", value=msg.param1)
            
        elif command == MavCommand.SET_SPEED_SURFACE:
            status.update_status(key="max_speed_surface", value=msg.param1)
        
        elif command == MavCommand.SET_LIGHT:
            status.update_status(key="light", value=msg.param1)
            
        elif command == MavCommand.SET_CAMERA:
            status.update_status(key="camera", value=msg.param1)
        
        elif command == MavCommand.START_MAG_CALIBRATION:
            calibrate.calibrate_mag()
    
    def handle_received_msg(self, msg) -> None:
        if msg is None:
            return

        if msg.get_type() == "COMMAND_LONG":
            try:
                command = MavCommand(msg.command)
                logger.info(f"Received MAVLink command: {command.name} (ID: {msg.command}), Params: {msg.param1}, {msg.param2}, {msg.param3}, {msg.param4}, {msg.param5}, {msg.param6}, {msg.param7}")

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

    global controller
    
    if controller is None:
        controller = MavlinkController(receive_port, send_ip, send_port)
    
    return controller.start()

def stop_mavlink():
    global controller
    
    if controller is not None:
        controller.stop()

def signal_handler(sig, frame) -> None:
    logger.info("Received shutdown signal, stopping MAVLink connections...")
    stop_mavlink()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if init_mavlink():
        logger.info("MAVLink communication started successfully")
        
        try:
            # Keep main process running
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Manual shutdown initiated...")
        finally:
            stop_mavlink()
    else:
        logger.error("Failed to initialize MAVLink communication")
        sys.exit(1)