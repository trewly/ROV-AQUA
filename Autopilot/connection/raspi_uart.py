import serial
import threading
import time
from typing import Optional, List
from enum import IntEnum
import atexit

_UART_INITIALIZED = False
_UART_LOCK = threading.RLock()

from Autopilot.utils.raspi_logger import LOG

class UARTCommand(IntEnum):
    SET_SPEED_RIGHT = 0
    SET_SPEED_LEFT = 1
    SET_SPEED_RIGHT_DEPTH = 2
    SET_SPEED_LEFT_DEPTH = 3

    INTIALIZE = 4
    STOP = 5
    FORWARD = 6
    BACKWARD = 7
    LEFT = 8
    RIGHT = 9
    SURFACE = 10
    DIVE = 11
    STOP_ALL = 12

    SET_SPEED_FORWARD = 13
    SET_SPEED_BACKWARD = 14
    SET_SPEED_SURFACE = 15
    SET_SPEED_DIVE = 16

class UARTController:
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        with _UART_LOCK:
            if cls._instance is None:
                cls._instance = super(UARTController, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance
    
    def __init__(self, 
                 port="/dev/ttyAMA0", 
                 baudrate=115200, 
                 timeout=1.0):
        if self._initialized:
            return
            
        LOG.info(f"Initializing UART controller on {port} at {baudrate} baud")
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self.serial_connection: Optional[serial.Serial] = None
        self.running = False
        self.threads: List[threading.Thread] = []
        
        self._lock = threading.RLock()
        self._initialized = True
    
    def initialize(self):
        global _UART_INITIALIZED
        
        with self._lock:
            if self.running:
                LOG.info("UART controller already running")
                return True
            
            if _UART_INITIALIZED:
                LOG.info("UART already initialized in another part of the application")
                return True
            
            try:
                self.serial_connection = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                
                if not self.serial_connection.is_open:
                    self.serial_connection.open()
                    
                self.running = True
                _UART_INITIALIZED = True
                LOG.info("UART controller started successfully")
                return True
                
            except Exception as e:
                LOG.error(f"Failed to initialize UART: {e}")
                self.shutdown()
                return False
    
    def read_line(self):
        if not self.running or not self.serial_connection:
            LOG.warning("Cannot read: UART not initialized")
            return None
            
        try:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline()
                if data:
                    return data.decode('utf-8').strip()
            return None
        except Exception as e:
            LOG.error(f"Error reading from UART: {e}")
            return None
    
    def read_command(self):
        line = self.read_line()
        if not line:
            return None

        if ":" in line:
            parts = line.split(":")
            if len(parts) == 2:
                try:
                    command_value = int(parts[0])
                    param_value = int(parts[1])
                    LOG.info(f"Decoded command: {command_value}, param: {param_value}")
                    return (command_value, param_value)
                except ValueError:
                    LOG.warning(f"Invalid number format in command: {line}")
        
        return None
    
    def shutdown(self):
        global _UART_INITIALIZED
        
        LOG.info("Shutting down UART controller...")
        
        with self._lock:
            self.running = False
            _UART_INITIALIZED = False
        
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=2)
                if thread.is_alive():
                    LOG.warning(f"Thread {thread.name} did not terminate gracefully")
        
        self.threads = []
        
        if self.serial_connection:
            try:
                if self.serial_connection.is_open:
                    self.serial_connection.close()
            except Exception as e:
                LOG.error(f"Error closing serial connection: {e}")
            self.serial_connection = None
            
        LOG.info("UART controller stopped")
    
    def send_data(self, data):
        with self._lock:
            if not self.running or not self.serial_connection:
                if not self.initialize():
                    LOG.error("Cannot send data: UART initialization failed")
                    return False
                
            try:
                bytes_written = self.serial_connection.write(data)
                self.serial_connection.flush()
                LOG.debug(f"Sent {bytes_written} bytes over UART")
                return bytes_written == len(data)
            except Exception as e:
                LOG.error(f"Error sending data: {e}")
                return False
    
    def send_string(self, data):
        return self.send_data(data.encode('utf-8'))
    
    def send_command(self, command, param=0):
        cmd_map = {
            "stop": UARTCommand.STOP,
            "forward": UARTCommand.FORWARD,
            "backward": UARTCommand.BACKWARD,
            "left": UARTCommand.LEFT,
            "right": UARTCommand.RIGHT,
            "surface": UARTCommand.SURFACE,
            "dive": UARTCommand.DIVE,
            "stop_all": UARTCommand.STOP_ALL,
            
            "set_speed_left": UARTCommand.SET_SPEED_LEFT,
            "set_speed_right": UARTCommand.SET_SPEED_RIGHT,
            "set_speed_left_depth": UARTCommand.SET_SPEED_LEFT_DEPTH,
            "set_speed_right_depth": UARTCommand.SET_SPEED_RIGHT_DEPTH,
            "set_speed_forward": UARTCommand.SET_SPEED_FORWARD,
            "set_speed_backward": UARTCommand.SET_SPEED_BACKWARD,
            "set_speed_surface": UARTCommand.SET_SPEED_SURFACE,
            "set_speed_dive": UARTCommand.SET_SPEED_DIVE,
            
            "initialize": UARTCommand.INTIALIZE,
        }
        
        if isinstance(command, str):
            command = command.lower()
            if command not in cmd_map:
                LOG.warning(f"Unknown command: {command}")
                return False
            command = cmd_map[command]
                
        command_str = f"{int(command)}:{param}\n"
        
        return self.send_string(command_str)

    def stop(self):
        return self.send_command(UARTCommand.STOP)

    def forward(self):
        return self.send_command(UARTCommand.FORWARD)

    def backward(self):
        return self.send_command(UARTCommand.BACKWARD)

    def left(self):
        return self.send_command(UARTCommand.LEFT)

    def right(self):
        return self.send_command(UARTCommand.RIGHT)

    def surface(self):
        return self.send_command(UARTCommand.SURFACE)

    def dive(self):
        return self.send_command(UARTCommand.DIVE)

    def set_speed_left(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_LEFT, speed)

    def set_speed_right(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_RIGHT, speed)

    def set_speed_left_depth(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_LEFT_DEPTH, speed)

    def set_speed_right_depth(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_RIGHT_DEPTH, speed)

    def set_speed_forward(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_FORWARD, speed)

    def set_speed_backward(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_BACKWARD, speed)

    def set_speed_surface(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_SURFACE, speed)

    def set_speed_dive(self, speed):
        return self.send_command(UARTCommand.SET_SPEED_DIVE, speed)

    def initialize_motors(self):
        return self.send_command(UARTCommand.INTIALIZE)

UART = UARTController()

def initialize():
    with _UART_LOCK:
        if not _UART_INITIALIZED:
            UART.initialize()

initialize()

atexit.register(UART.shutdown)