import serial
import threading
import time
from typing import Optional, List, Callable
from enum import IntEnum
from gpiozero import DigitalInputDevice

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
                 rx_pin=15):
        print(f"Initializing UART controller on {port} at {baudrate} baud")
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.rx_pin = rx_pin
        
        self.serial_connection: Optional[serial.Serial] = None
        self.running = False
        self.threads: List[threading.Thread] = []
        
        self.receive_callback: Optional[Callable[[bytes], None]] = None
        self.data_ready = threading.Event()
        
        self._lock = threading.RLock()
        self.rx_sensor = None
    
    def initialize(self):
        with self._lock:
            if self.running:
                print("UART controller already running")
                return False
            
            try:
                self.rx_sensor = DigitalInputDevice(
                    pin=self.rx_pin,
                    pull_up=True,
                    bounce_time=0.001
                )
                
                self.rx_sensor.when_activated = self._uart_interrupt_handler
                
                self.serial_connection = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                
                if not self.serial_connection.is_open:
                    self.serial_connection.open()
                    
                self.running = True
                
                processor_thread = threading.Thread(
                    target=self._process_data,
                    daemon=True,
                    name="UART-Data-Processor"
                )
                processor_thread.start()
                self.threads.append(processor_thread)
                
                print("UART controller started successfully with interrupt-based reception")
                return True
                
            except Exception as e:
                print(f"Failed to initialize UART: {e}")
                self.shutdown()
                return False
    
    def _uart_interrupt_handler(self):
        self.data_ready.set()
    
    def _process_data(self):
        print("UART data processor started")
        
        while self.running:
            if self.data_ready.wait(timeout=0.1):
                self.data_ready.clear()
                
                try:
                    if self.serial_connection and self.serial_connection.in_waiting > 0:
                        data = self.serial_connection.read(self.serial_connection.in_waiting)
                        
                        if data:
                            print(f"Received {len(data)} bytes over UART (interrupt)")
                            if self.receive_callback:
                                try:
                                    self.receive_callback(data)
                                except Exception as e:
                                    print(f"Error in receive callback: {e}")
                                    
                except Exception as e:
                    print(f"Error reading UART data: {e}")
    
    def shutdown(self):
        print("Shutting down UART controller...")
        
        with self._lock:
            self.running = False
        
        if self.rx_sensor:
            self.rx_sensor.close()
            self.rx_sensor = None
        
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
        baudrate=115200,
        rx_pin=155
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