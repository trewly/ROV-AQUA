import time
import threading
import sys
import os
import atexit
from enum import IntEnum
from typing import List
from pymavlink import mavutil

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Autopilot.system_info.sensor import raspi_sensor_read as sensor
from Autopilot.system_info.status import raspi_status as status
from Autopilot.config import raspi_config as config
from Autopilot.controller.motor import raspi_motor_control as rov
from Autopilot.controller.camera import raspi_camera as camera
from Autopilot.utils.raspi_logger import LOG
from Autopilot.connection.raspi_uart import UART

_MAV_INITIALIZED = False
_MAV_LOCK = threading.RLock()

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
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        with _MAV_LOCK:
            if cls._instance is None:
                cls._instance = super(MavlinkController, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance
            
    def __init__(self, pc_ip="169.254.54.121", receive_port=5000, send_port=5001, status_send_interval=0.01,
                  heartbeat_timeout=10.0):
        if self._initialized:
            return
            
        LOG.info("Initializing MAVLink controller")
            
        self.receive_port = receive_port
        self.pc_ip = pc_ip
        self.send_port = send_port
        
        self.status_send_interval = status_send_interval
        self.heartbeat_timeout = heartbeat_timeout
        self.last_heartbeat = time.time()
        
        self.running = False
        self.threads: List[threading.Thread] = []
        self.connection_lost_reported = False
        
        self.connection_lost = False
        self.connection_lost_time = 0
        self.emergency_mode_active = False
        
        self.emergency_surface_timeout = 240
        
        self.receiver = None
        self.transmitter = None
        
        self.status_params = [
            "roll", "pitch", "heading", "internal_temp", "external_temp", "depth", 
            "horizontal_velocity", "vertical_velocity"
        ]
        
        self._lock = threading.RLock()
        self._initialized = True

    def _initialize_connection(self):
        global _MAV_INITIALIZED
        
        with self._lock:
            if self.running:
                LOG.warning("MAVLink controller already running")
                return False
                
            if _MAV_INITIALIZED:
                LOG.warning("MAVLink already initialized in another part of the application")
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
            
            self.transmitter = mavutil.mavlink_connection(f"udpout:{self.pc_ip}:{self.send_port}")
            transmitter_thread = threading.Thread(
                target=self.send_status_updates, 
                daemon=True,
                name="MAVLink-Transmitter"
            )
            transmitter_thread.start()
            self.threads.append(transmitter_thread)

            _MAV_INITIALIZED = True
            LOG.info("MAVLink controller started successfully")

            return True
            
        except Exception as e:
            LOG.error(f"Failed to start MAVLink controller: {e}", exc_info=True)
            self.shutdown()
            return False

    def shutdown(self):
        global _MAV_INITIALIZED
        
        LOG.info("Stopping MAVLink controller...")
        
        with self._lock:
            self.running = False
            _MAV_INITIALIZED = False
        
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=2)
                if thread.is_alive():
                    LOG.warning(f"Thread {thread.name} did not terminate gracefully")
        
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
            
        LOG.info("MAVLink controller stopped")

    def _get_command_handler(self, command):
        command_handlers = {
            MavCommand.SURFACE: rov.surface,
            MavCommand.DIVE: rov.dive,
            MavCommand.LEFT: rov.turn_left,
            MavCommand.RIGHT: rov.turn_right,
            MavCommand.FORWARD: rov.move_forward,
            MavCommand.BACKWARD: rov.move_backward,
            MavCommand.STOP: rov.stop_all,
        }
        
        return command_handlers.get(command)

    def _handle_manual_command(self, command):
            handler = self._get_command_handler(command)
            if handler:
                try:
                    handler()
                    LOG.info(f"Executed manual command: {command}")
                except Exception as e:
                    LOG.error(f"Error executing manual command {command}: {e}")
            else:
                LOG.warning(f"Unknown manual command: {command}")

    def _handle_mode_commands(self, msg):
        command = msg.command
        
        if command == MavCommand.SET_MANUAL:
            status.update_multiple({
                "mode": "manual",
                "auto_heading": 0,
                "auto_depth": 0
            })
            rov.stop_all()
            
        elif command == MavCommand.SET_AUTO_HEADING:
            heading = max(0, min(360, msg.param1))
            status.update_multiple({
                "auto_heading": 1,
                "target_heading": heading,
                "mode": "auto_heading"
            })
            
        elif command == MavCommand.SET_AUTO_DEPTH:
            depth = max(0, min(10, msg.param1))
            status.update_multiple({
                "auto_depth": 1,
                "target_depth": depth,
                "mode": "auto_depth"
            })
            
    
    def _handle_configuration_commands(self, msg):
        command = msg.command
        
        if command == MavCommand.SET_SPEED_FORWARD:
            UART.set_speed_forward(msg.param1)
            config.update_config(key="max_speed_forward", value=msg.param1)
            
        elif command == MavCommand.SET_SPEED_BACKWARD:
            UART.set_speed_backward(msg.param1)
            config.update_config(key="max_speed_backward", value=-msg.param1)
            
        elif command == MavCommand.SET_SPEED_DIVE:
            UART.set_speed_dive(msg.param1)
            config.update_config(key="max_speed_dive", value=-msg.param1)
            
        elif command == MavCommand.SET_SPEED_SURFACE:
            UART.set_speed_surface(msg.param1)
            config.update_config(key="max_speed_surface", value=msg.param1)

        elif command == MavCommand.SET_PID_YAW:
            config.update_multiple({
                "Kp_yaw": msg.param1,
                "Ki_yaw": msg.param2,
                "Kd_yaw": msg.param3
            })
        
        elif command == MavCommand.SET_PID_DEPTH:
            config.update_multiple({
                "Kp_depth": msg.param1,
                "Ki_depth": msg.param2,
                "Kd_depth": msg.param3
            })
        
        elif command == MavCommand.SET_PID_AUTOHEADING:
            config.update_multiple({
                "Kp_autoheading": msg.param1,
                "Ki_autoheading": msg.param2,
                "Kd_autoheading": msg.param3
            })
        
        elif command == MavCommand.SET_LIGHT:
            status.update_status(key="light", value=msg.param1)
           
        elif command == MavCommand.SET_CAMERA:
            status.update_status(key="camera", value=msg.param1)
        
        elif command == MavCommand.START_MAG_CALIBRATION:
            sensor.initialize_sensors()
            time.sleep(0.1)
            rov.initialize_motors()
    
            try:
                self.transmitter.mav.param_value_send(
                    "calibrated".encode("ascii"),
                    1.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
                    0,
                    1
                )
            except Exception as e:
                LOG.error(f"Failed to send calibration status: {e}")

            time.sleep(0.1)
            sensor.sensor_fusion.start_update()

        elif command == MavCommand.START_CAMERA_STREAM:
            camera.start_stream()

    def handle_received_msg(self, msg):
        if msg is None:
            return

        if msg.get_type() == "COMMAND_LONG":
            try:
                command = MavCommand(msg.command)
                
                if self.emergency_mode_active:
                    if command == MavCommand.SET_MANUAL:
                        LOG.info("Emergency mode: Manual control restored by operator")
                        self.emergency_mode_active = False
                        status.update_status(key="mode", value="manual")
                        rov.stop_all()
                    else:
                        LOG.warning(f"Emergency mode active: Ignoring command {command}")
                        return
                    
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
                        LOG.info(f"Mode changed: Manual control")

                    elif command == MavCommand.SET_AUTO_HEADING:
                        LOG.info(f"Mode changed: Auto heading with target {msg.param1}°")

                    elif command == MavCommand.SET_AUTO_DEPTH:
                        LOG.info(f"Mode changed: Auto depth with target {msg.param1}m")
                
                    elif command == MavCommand.SET_SPEED_FORWARD:
                        LOG.info(f"Forward speed set to {msg.param1}%")

                    elif command == MavCommand.SET_SPEED_BACKWARD:
                        LOG.info(f"Backward speed set to {msg.param1}%") 

                    elif command == MavCommand.SET_SPEED_DIVE:
                        LOG.info(f"Dive speed set to {msg.param1}%")

                    elif command == MavCommand.SET_SPEED_SURFACE:
                        LOG.info(f"Surface speed set to {msg.param1}%")

                    elif command == MavCommand.SET_LIGHT:
                        LOG.info(f"Light set to on")

                    elif command == MavCommand.SET_CAMERA:
                        LOG.info(f"Camera set to on")
                        
                    elif command == MavCommand.START_MAG_CALIBRATION:
                        LOG.info(f"Starting magnetometer calibration")
                
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
                LOG.warning(f"Received unknown command ID: {msg.command}")
            except Exception as e:
                LOG.error(f"Error handling command: {e}", exc_info=True)

    def _handle_connection_lost(self):
        if not self.connection_lost_reported:
            LOG.warning("Connection lost with ground station, initiating emergency surface protocol")
            self.connection_lost_reported = True
            self.connection_lost = True
            self.connection_lost_time = time.time()
            self.emergency_mode_active = True
            
            status_update = {
                "disconnect": 1,
                "mode": "emergency",
                "auto_heading": 0,
                "auto_depth": 0
            }
            status.update_multiple(status_update)
            
            current_depth = status.read_status(key="depth", default=0)
            if current_depth > 0.2:
                LOG.info("Emergency protocol: Initiating surface command")
                rov.surface()
            else:
                LOG.info("Emergency protocol: Already at surface level, stopping all motors")
                rov.stop_all()

    def _handle_connection_restored(self) -> None:
        if self.connection_lost:
            LOG.info("Connection with ground station restored")
            self.connection_lost = False
            self.connection_lost_reported = False
            self.emergency_mode_active = False
            
            status.update_multiple({
                "disconnect": 0,
                "mode": "manual"
            })
            
            rov.stop_all()

    def receive_messages(self) -> None:
        LOG.info(f"MAVLink receiver started on port {self.receive_port}")
        
        while self.running:
            try:
                msg = self.receiver.recv_match(blocking=True, timeout=1)
                if msg:
                    with self._lock:
                        # Log mọi message nhận được để debug
                        LOG.debug(f"Received message: {msg.get_type()}")
                        
                        if self.connection_lost:
                            self._handle_connection_restored()
                        
                        status.update_status(key="disconnect", value=0)
                        
                        if msg.get_type() == "HEARTBEAT":
                            LOG.debug("HEARTBEAT received, updating last_heartbeat time")
                            self.last_heartbeat = time.time()
                        elif msg.get_type() == "COMMAND_LONG":
                            self.handle_received_msg(msg)
                else:
                    with self._lock:
                        current_time = time.time()
                        if current_time - self.last_heartbeat > self.heartbeat_timeout:
                            if not self.connection_lost:
                                self._handle_connection_lost()
                            elif self.emergency_mode_active:
                                emergency_elapsed = current_time - self.connection_lost_time
                                
                                if int(emergency_elapsed) % 30 == 0:
                                    LOG.info(f"Emergency mode active for {int(emergency_elapsed)}s")
                                    
                                    current_depth = status.read_status(key="depth", default=0)
                                    if current_depth > 0.3:
                                        LOG.info(f"Still submerged at depth {current_depth}m, continuing surface maneuver")
                                        rov.surface()
                                        
                                if emergency_elapsed > self.emergency_surface_timeout:
                                    if status.read_status(key="mode") != "emergency_shutdown":
                                        LOG.warning("Emergency surface timeout reached, shutting down motors to conserve battery")
                                        rov.stop_all()
                                        status.update_status(key="mode", value="emergency_shutdown")
            except ConnectionError as e:
                LOG.error(f"Connection error in receiver: {e}")
            except Exception as e:
                LOG.error(f"Error in receiver thread: {e}", exc_info=True)

    def send_status_updates(self):
        LOG.info(f"MAVLink transmitter sending to {self.pc_ip}:{self.send_port}")
        
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        last_send_time = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                if current_time - last_send_time >= self.status_send_interval:
                    last_send_time = current_time
                    
                    status_data = status.read_multiple_status(self.status_params)
                    
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
                            LOG.warning(f"Invalid value for {key}: {e}")
                        except Exception as e:
                            LOG.error(f"Error sending {key}: {e}")
                    
                    # self.transmitter.mav.heartbeat_send(
                    #     mavutil.mavlink.MAV_TYPE_SUBMARINE,
                    #     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    #     0,
                    #     0,
                    #     mavutil.mavlink.MAV_STATE_ACTIVE
                    # )
                
            except Exception as e:
                LOG.error(f"Error in transmitter thread: {e}", exc_info=True)



MAV = MavlinkController()

def initialize():
    with _MAV_LOCK:
        if not _MAV_INITIALIZED:
            MAV._initialize_connection()

initialize()

def cleanup():
    MAV.shutdown()
    for handler in LOG.handlers:
        handler.flush()
        handler.close()

atexit.register(cleanup)

if __name__ == "__main__":
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        MAV.shutdown()