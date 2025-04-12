import time
import threading
import sys
import os
import atexit
from pymavlink import mavutil

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Mission_planner.status import pc_status as status
from Mission_planner.utils.pc_logger import LOG

class MavCommands:
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
    def __init__(self, raspi_ip="169.254.54.120", send_port=5000, receive_port=5001, 
                heartbeat_interval=1.0, connection_timeout=10.0):
        self.raspi_ip = raspi_ip
        self.send_port = send_port
        self.receive_port = receive_port
        self.heartbeat_interval = heartbeat_interval
        self.connection_timeout = connection_timeout
        
        self.last_message_time = time.time()
        self._is_connected = False
        self.running = False
        self.command_counter = 0
        self.connection_initialized = False
        
        self.heartbeat_thread = None
        self.receive_thread = None
        
        self.send_lock = threading.Lock()
        
        for attr in dir(MavCommands):
            if not attr.startswith('__'):
                setattr(self, attr, getattr(MavCommands, attr))
                
    def _initialize_connections(self):
        try:
            self.master_send = mavutil.mavlink_connection(f"udpout:{self.raspi_ip}:{self.send_port}")
            LOG.info(f"Sender initialized to {self.raspi_ip}:{self.send_port}")
            
            self.master_receive = mavutil.mavlink_connection(f"udpin:0.0.0.0:{self.receive_port}")
            LOG.info(f"Receiver initialized on port {self.receive_port}")
            
            self.heartbeat_thread = threading.Thread(
                target=self._send_heartbeat_loop, 
                daemon=True,
                name="HeartbeatThread"
            )
            self.heartbeat_thread.start()
            
            self.receive_thread = threading.Thread(
                target=self._receive_message_loop, 
                daemon=True,
                name="ReceiveThread"
            )
            self.receive_thread.start()
            
            self.connection_initialized = True
            self.running = True
            self._is_connected = True
            LOG.info("MAVLink controller initialized successfully")
            return True
        except Exception as e:
            self.running = False
            LOG.error(f"Failed to initialize MAVLink: {e}")
            return False
        
    def shutdown(self):
        if self._is_connected:
            LOG.info("Shutting down MAVLink controller")
            self.running = False
            
            if hasattr(self, 'heartbeat_thread') and self.heartbeat_thread and self.heartbeat_thread.is_alive():
                self.heartbeat_thread.join(timeout=2)
            if hasattr(self, 'receive_thread') and self.receive_thread and self.receive_thread.is_alive():
                self.receive_thread.join(timeout=2)
                
            LOG.info("MAVLink controller shutdown complete")
            LOG.info("=" * 50)

    def _send_heartbeat_loop(self):
        LOG.info("Heartbeat thread started")
        while self.running:
            try:
                with self.send_lock:
                    if self.master_send.target_system == 0:
                        self.master_send.target_system = 1
                    if self.master_send.target_component == 0:
                        self.master_send.target_component = 1
                        
                    self.master_send.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                        0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                self._is_connected = True
                time.sleep(self.heartbeat_interval)
            except Exception as e:
                LOG.error(f"Error sending heartbeat: {e}")
                time.sleep(1)

    def _receive_message_loop(self):
        LOG.info("Receive thread started")
        while self.running:
            try:
                msg = self.master_receive.recv_match(blocking=True, timeout=1)
                current_time = time.time()
                
                if msg is not None:
                    if not self._is_connected:
                        LOG.info("Connection established")
                        self._is_connected = True
                    
                    status.update_status("disconnected", 0)
                    self.last_message_time = current_time
                    
                    self._process_message(msg)
                else:
                    if current_time - self.last_message_time > self.connection_timeout:
                        if self._is_connected:
                            LOG.warning("Connection lost!")
                            status.update_status("disconnected", 1)
                            self._is_connected = False
            except Exception as e:
                LOG.error(f"Error receiving message: {e}")
                time.sleep(1)

    def _process_message(self, msg):
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            LOG.debug(f"HEARTBEAT: sys_id={msg.get_srcSystem()}, comp_id={msg.get_srcComponent()}, type={msg.type}")
        elif msg_type == "PARAM_VALUE":
            try:
                if isinstance(msg.param_id, bytes):
                    param_id = msg.param_id.decode('ascii').rstrip('\x00')
                else:
                    param_id = str(msg.param_id).rstrip('\x00')
                    
                param_value = msg.param_value
                status.update_status(param_id, param_value)
                
            except Exception as e:
                LOG.error(f"Error processing parameter: {e}")
        else:
            try:
                msg_dict = msg.to_dict()
                log_msg = f"{msg_type}: " + ", ".join([f"{k}={v}" for k, v in msg_dict.items() 
                                                   if k not in ('mavpackettype', 'time_usec')])
                LOG.info(log_msg)
            except:
                LOG.info(f"Received message of type: {msg_type}")
    
    def is_connected(self):
        return self._is_connected
    
    def close(self):
        if not self.running:
            LOG.info("MAVLink controller is already stopped")
            return True
            
        self.shutdown()
        return True

    def send_command(self, command, param1=0, param2=0, param3=0, 
                   param4=0, param5=0, param6=0, param7=0, confirmation=0):
        try:
            if not self.running or not self._is_connected:
                LOG.error(f"Cannot send command {command}: Not connected")
                return False
                
            with self.send_lock:
                self.master_send.mav.command_long_send(
                    self.master_send.target_system,
                    self.master_send.target_component,
                    command,
                    confirmation,
                    param1, param2, param3, param4, param5, param6, param7
                )
            self.command_counter += 1
            LOG.debug(f"Sent command: {command} with params: {param1}, {param2}, {param3}")
            return True
        except Exception as e:
            LOG.error(f"Error sending command {command}: {e}")
            return False

    def send_control_cmd(self, cmd):
        return self.send_command(cmd)

    def set_manual_mode(self):
        LOG.info("Setting manual mode")
        return self.send_command(MavCommands.SET_MANUAL)

    def set_auto_heading(self, heading):
        heading = max(0, min(360, heading))
        LOG.info(f"Setting auto heading: {heading}°")
        return self.send_command(MavCommands.SET_AUTO_HEADING, param1=heading)

    def set_auto_depth(self, depth):
        depth = max(0, min(10, depth))
        LOG.info(f"Setting auto depth: {depth}m")
        return self.send_command(MavCommands.SET_AUTO_DEPTH, param1=depth)

    def set_max_speed_forward(self, max_speed):
        max_speed = max(1, min(100, max_speed))
        LOG.info(f"Setting max forward speed: {max_speed}")
        return self.send_command(MavCommands.SET_SPEED_FORWARD, param1=max_speed)

    def set_max_speed_backward(self, max_speed):
        max_speed = max(-100, min(-1, max_speed))
        LOG.info(f"Setting max backward speed: {max_speed}")
        return self.send_command(MavCommands.SET_SPEED_BACKWARD, param1=max_speed)

    def set_max_speed_dive(self, max_speed):
        max_speed = max(-100, min(-1, max_speed))
        LOG.info(f"Setting max dive speed: {max_speed}")
        return self.send_command(MavCommands.SET_SPEED_DIVE, param1=max_speed)
    
    def set_max_speed_surface(self, max_speed):
        max_speed = max(1, min(100, max_speed))
        LOG.info(f"Setting max surface speed: {max_speed}")
        return self.send_command(MavCommands.SET_SPEED_SURFACE, param1=max_speed)
        
    def set_pid_yaw(self, Kp, Ki, Kd):
        LOG.info(f"Setting PID_YAW: Kp={Kp}, Ki={Ki}, Kd={Kd}")
        return self.send_command(MavCommands.SET_PID_YAW, param1=Kp, param2=Ki, param3=Kd)
    
    def set_pid_autoheading(self, Kp, Ki, Kd):
        LOG.info(f"Setting PID_AUTOHEADING: Kp={Kp}, Ki={Ki}, Kd={Kd}")
        return self.send_command(MavCommands.SET_PID_AUTOHEADING, param1=Kp, param2=Ki, param3=Kd)
    
    def set_pid_depth(self, Kp, Ki, Kd):
        LOG.info(f"Setting PID_DEPTH: Kp={Kp}, Ki={Ki}, Kd={Kd}")
        return self.send_command(MavCommands.SET_PID_DEPTH, param1=Kp, param2=Ki, param3=Kd)

    def set_light(self):
        LOG.info(f"Setting light on")
        return self.send_command(MavCommands.SET_LIGHT)

    def set_camera(self):
        LOG.info(f"Setting camera on")
        return self.send_command(MavCommands.SET_CAMERA)

    def start_mag_calibration(self):
        LOG.info("Starting magnetometer calibration")
        return self.send_command(MavCommands.START_MAG_CALIBRATION)
    
    def start_camera_stream(self):
        LOG.info("Starting camera")
        return self.send_command(MavCommands.START_CAMERA_STREAM)

    def get_status(self, status_id):
        try:
            if not self.running or not self._is_connected:
                LOG.error(f"Cannot request status {status_id}: Not connected")
                return False
                
            with self.send_lock:
                self.master_send.mav.param_request_read_send(
                    self.master_send.target_system,
                    self.master_send.target_component,
                    status_id.encode('utf-8'),
                    -1
                )
            LOG.debug(f"Requested status: {status_id}")
            return True
        except Exception as e:
            LOG.error(f"Error requesting status {status_id}: {e}")
            return False

MAV = MavlinkController()

def cleanup():
    MAV.shutdown()
    for handler in LOG.handlers:
        handler.flush()
        handler.close()

atexit.register(cleanup)

if __name__ == "__main__":
    print("MAVLink controller test")
    print("Press Ctrl+C to exit")
    
    try:
        MAV._initialize_connections()
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        MAV.shutdown()