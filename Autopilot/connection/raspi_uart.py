import serial
import threading
import time
from typing import Optional, List, Callable
from enum import IntEnum

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
    def __init__(self, 
                 port="/dev/ttyAMA0", 
                 baudrate=115200, 
                 timeout=1.0,
                 read_interval=0.01):
        print(f"Initializing UART controller on {port} at {baudrate} baud")
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.read_interval = read_interval
        
        self.serial_connection: Optional[serial.Serial] = None
        self.running = False
        self.threads: List[threading.Thread] = []
        
        self.receive_callback: Optional[Callable[[bytes], None]] = None
        
        self._lock = threading.RLock()
    
    def initialize(self):
        with self._lock:
            if self.running:
                print("UART controller already running")
                return False
            
            try:
                self.serial_connection = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                
                if not self.serial_connection.is_open:
                    self.serial_connection.open()
                    
                self.running = True
                
                receiver_thread = threading.Thread(
                    target=self._receive_data,
                    daemon=True,
                    name="UART-Receiver"
                )
                receiver_thread.start()
                self.threads.append(receiver_thread)
                
                print("UART controller started successfully")
                return True
                
            except Exception as e:
                print(f"Failed to initialize UART: {e}")
                self.shutdown()
                return False
    
    def shutdown(self):
        print("Shutting down UART controller...")
        
        with self._lock:
            self.running = False
        
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=2)
                if thread.is_alive():
                    print(f"Thread {thread.name} did not terminate gracefully")
        
        self.threads = []
        
        if self.serial_connection:
            try:
                if self.serial_connection.is_open:
                    self.serial_connection.close()
            except Exception as e:
                print(f"Error closing serial connection: {e}")
            self.serial_connection = None
            
        print("UART controller stopped")
    
    def send_data(self, data):
        with self._lock:
            if not self.running or not self.serial_connection:
                print("Cannot send data: UART not initialized")
                return False
                
            try:
                bytes_written = self.serial_connection.write(data)
                self.serial_connection.flush()
                print(f"Sent {bytes_written} bytes over UART")
                return bytes_written == len(data)
            except Exception as e:
                print(f"Error sending data: {e}")
                return False
    
    def send_string(self, data):
        return self.send_data(data.encode('utf-8'))
    
    def register_receive_callback(self, callback):
        self.receive_callback = callback
    
    def _receive_data(self):
        print(f"UART receiver started on {self.port}")
        
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    
                    if data:
                        print(f"Received {len(data)} bytes over UART")
                        
                        if self.receive_callback:
                            try:
                                self.receive_callback(data)
                            except Exception as e:
                                print(f"Error in receive callback: {e}")
                
                time.sleep(self.read_interval)
                
            except Exception as e:
                print(f"Error receiving data: {e}")
                time.sleep(1.0)
                
                with self._lock:
                    if self.serial_connection and not self.serial_connection.is_open:
                        try:
                            print("Attempting to reopen serial connection...")
                            self.serial_connection.open()
                        except Exception as e:
                            print(f"Failed to reopen serial connection: {e}")

    def send_command(self, command, param=0):
        if command == "set_speed_left":
            command = UARTCommand.SET_SPEED_LEFT

        elif command == "set_speed_right":
            command = UARTCommand.SET_SPEED_RIGHT

        elif command == "set_speed_left_depth":
            command = UARTCommand.SET_SPEED_LEFT_DEPTH

        elif command == "set_speed_right_depth":
            command = UARTCommand.SET_SPEED_RIGHT_DEPTH

        elif command == "set_speed_forward":
            command = UARTCommand.SET_SPEED_FORWARD

        elif command == "set_speed_backward":
            command = UARTCommand.SET_SPEED_BACKWARD
        
        elif command == "set_speed_surface":
            command = UARTCommand.SET_SPEED_SURFACE

        elif command == "set_speed_dive":
            command = UARTCommand.SET_SPEED_DIVE

        elif command == "stop":
            command = UARTCommand.STOP

        elif command == "forward":
            command = UARTCommand.FORWARD

        elif command == "backward":
            command = UARTCommand.BACKWARD

        elif command == "left":
            command = UARTCommand.LEFT

        elif command == "right":
            command = UARTCommand.RIGHT

        elif command == "surface":
            command = UARTCommand.SURFACE

        elif command == "dive":
            command = UARTCommand.DIVE

        elif command == "stop_all":
            command = UARTCommand.STOP_ALL

            param = int(param)
        elif command == "initialize":
            command = UARTCommand.INTIALIZE

        else:
            print(f"Unknown command: {command}")
            return False
        
        param = int(param)
        command_bytes = bytes([int(command), param])
        return self.send_data(command_bytes)

UART = UARTController()

if __name__ == "__main__":
    uart = UARTController(
        port="/dev/ttyAMA0",
        baudrate=115200
    )
    
    def on_data_received(data: bytes):
        try:
            message = data.decode('utf-8')
            print(f"Received message: {message}")
        except UnicodeDecodeError:
            print(f"Received binary data: {data.hex()}")
    
    uart.register_receive_callback(on_data_received)
    
    if uart.initialize():
        try:
            uart.send_string("Hello from Pi5!\n")
            
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            uart.shutdown()