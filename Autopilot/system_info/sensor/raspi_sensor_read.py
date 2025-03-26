import smbus2 as smbus
import time
import math
import numpy as np
import threading

from Autopilot.system_info.status import raspi_status as status
from Autopilot.controller.utils.raspi_Filter import KalmanFilter, low_pass_filter

G = 9.80665

MPU9250_ADDR = 0x68

PWR_MGMT_1 = 0x6B      
PWR_MGMT_2 = 0x6C      
SMPLRT_DIV = 0x19      
CONFIG = 0x1A          
GYRO_CONFIG = 0x1B     
ACCEL_CONFIG = 0x1C    
ACCEL_CONFIG_2 = 0x1D  
INT_ENABLE = 0x38      

TEMP_OUT = 0x41

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

ACCEL_SCALE = 16384.0  
GYRO_SCALE = 131.0     

GYRO_RANGE_250DPS = 0x00  
GYRO_RANGE_500DPS = 0x08  
GYRO_RANGE_1000DPS = 0x10 
GYRO_RANGE_2000DPS = 0x18 

ACCEL_RANGE_2G = 0x00    
ACCEL_RANGE_4G = 0x08    
ACCEL_RANGE_8G = 0x10    
ACCEL_RANGE_16G = 0x18   

ACCEL_DLPF_41HZ = 0x03   
GYRO_DLPF_41HZ = 0x03    

current_velocity_x = 0.0
current_velocity_y = 0.0
current_velocity_z = 0.0
velocity_last_update = time.time()
velocity_reset_counter = 0

VELOCITY_RESET_THRESHOLD = 100  

ACCEL_ALPHA = 0.1  
VELOCITY_ALPHA = 0.8  

ACCEL_THRESHOLD = 0.05  

bus = smbus.SMBus(1)
kalman_pitch = KalmanFilter()
kalman_roll = KalmanFilter()
prev_time = time.time()
current_pitch = 0
current_roll = 0

filtered_accel_x = 0.0
filtered_accel_y = 0.0
filtered_accel_z = 0.0

sensor_initialized = False

def init_mpu9250():
    global sensor_initialized
    
    try:
        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x80)
        time.sleep(0.1)  
        
        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)  
        
        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x01)  
        
        bus.write_byte_data(MPU9250_ADDR, SMPLRT_DIV, 0x04)  
        
        bus.write_byte_data(MPU9250_ADDR, CONFIG, GYRO_DLPF_41HZ)  
        
        bus.write_byte_data(MPU9250_ADDR, GYRO_CONFIG, GYRO_RANGE_250DPS)  
        
        bus.write_byte_data(MPU9250_ADDR, ACCEL_CONFIG, ACCEL_RANGE_2G)  
        
        bus.write_byte_data(MPU9250_ADDR, ACCEL_CONFIG_2, ACCEL_DLPF_41HZ)  

        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_2, 0x00)
        
        
        who_am_i = bus.read_byte_data(MPU9250_ADDR, 0x75)
        if who_am_i == 0x71 or who_am_i == 0x73:  
            print(f"MPU9250/MPU9255 detected (0x{who_am_i:02X})")
            sensor_initialized = True
            return True
        else:
            print(f"Unknown device ID: 0x{who_am_i:02X}")
            return False
        
    except Exception as e:
        print(f"Error initializing MPU9250: {e}")
        return False

def check_sensor_connection():
    try:
        who_am_i = bus.read_byte_data(MPU9250_ADDR, 0x75)
        if who_am_i == 0x71 or who_am_i == 0x73:  
            return True
        return False
    except:
        return False

def read_word_mpu(register):
    global sensor_initialized

    try:
        if not sensor_initialized:
            if not init_mpu9250():
                return 0
                
        high = bus.read_byte_data(MPU9250_ADDR, register)
        low = bus.read_byte_data(MPU9250_ADDR, register + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value -= 0x10000
        return value
    except Exception as e:
        if "I/O" in str(e) and sensor_initialized:
            print("Sensor disconnected, trying to reinitialize...")
            sensor_initialized = False
            init_mpu9250()
        print(f"Error reading from MPU: {e}")
        return 0

def read_temp_data():
    temp = read_word_mpu(TEMP_OUT)
    temp = (temp / 340) + 36.53
    status.update_status(key="temp", value=temp)
    return temp

def read_gyro_data():
    gyro_x = read_word_mpu(GYRO_XOUT_H) / GYRO_SCALE
    gyro_y = read_word_mpu(GYRO_YOUT_H) / GYRO_SCALE
    gyro_z = read_word_mpu(GYRO_ZOUT_H) / GYRO_SCALE
    return gyro_x, gyro_y, gyro_z

def read_accel_data():
    accel_x = read_word_mpu(ACCEL_XOUT_H) / ACCEL_SCALE
    accel_y = read_word_mpu(ACCEL_YOUT_H) / ACCEL_SCALE
    accel_z = read_word_mpu(ACCEL_ZOUT_H) / ACCEL_SCALE
    return accel_x, accel_y, accel_z

def read_accel_gyro_data():
    accel_x, accel_y, accel_z = read_accel_data()
    gyro_x, gyro_y, gyro_z = read_gyro_data()
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def filter_acceleration(ax, ay, az):
    global filtered_accel_x, filtered_accel_y, filtered_accel_z
    
    filtered_accel_x = low_pass_filter(filtered_accel_x, ax, ACCEL_ALPHA)
    filtered_accel_y = low_pass_filter(filtered_accel_y, ay, ACCEL_ALPHA)
    filtered_accel_z = low_pass_filter(filtered_accel_z, az - 1.0, ACCEL_ALPHA)  
    
    if abs(filtered_accel_x) < ACCEL_THRESHOLD:
        filtered_accel_x = 0.0
    if abs(filtered_accel_y) < ACCEL_THRESHOLD:
        filtered_accel_y = 0.0
    if abs(filtered_accel_z) < ACCEL_THRESHOLD:
        filtered_accel_z = 0.0
    
    return filtered_accel_x, filtered_accel_y, filtered_accel_z

def calculate_pitch_roll(ax, ay, az):
    try:
        pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)
        roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        return pitch, roll
    except Exception as e:
        print(f"Error calculating pitch/roll: {e}")
        return 0, 0

def calculate_velocity():
    global current_velocity_x, current_velocity_y, current_velocity_z
    global velocity_last_update, velocity_reset_counter
    
    try:
        ax, ay, az = read_accel_data()
        
        filtered_ax, filtered_ay, filtered_az = filter_acceleration(ax, ay, az)
        
        current_time = time.time()
        dt = current_time - velocity_last_update
        velocity_last_update = current_time
        
        ax_ms2 = filtered_ax * G
        ay_ms2 = filtered_ay * G
        az_ms2 = filtered_az * G
        
        delta_vx = ax_ms2 * dt
        delta_vy = ay_ms2 * dt
        delta_vz = az_ms2 * dt
        
        current_velocity_x = low_pass_filter(current_velocity_x, current_velocity_x + delta_vx, VELOCITY_ALPHA)
        current_velocity_y = low_pass_filter(current_velocity_y, current_velocity_y + delta_vy, VELOCITY_ALPHA)
        current_velocity_z = low_pass_filter(current_velocity_z, current_velocity_z + delta_vz, VELOCITY_ALPHA)
        
        velocity_reset_counter += 1
        
        if velocity_reset_counter >= VELOCITY_RESET_THRESHOLD:
            velocity_reset_counter = 0
            
            current_velocity_x *= 0.5
            current_velocity_y *= 0.5
            current_velocity_z *= 0.5
        
        status.update_status(key="velocity_x", value=current_velocity_x)
        status.update_status(key="velocity_y", value=current_velocity_y)
        status.update_status(key="velocity_z", value=current_velocity_z)
        
        horizontal_velocity = math.sqrt(current_velocity_x**2 + current_velocity_y**2)
        status.update_status(key="horizontal_velocity", value=horizontal_velocity)
        
        status.update_status(key="vertical_velocity", value=current_velocity_z)
        
        return current_velocity_x, current_velocity_y, current_velocity_z
    
    except Exception as e:
        print(f"Error calculating velocity: {e}")
        return current_velocity_x, current_velocity_y, current_velocity_z

def get_velocity():
    return current_velocity_x, current_velocity_y, current_velocity_z

def get_orientation():
    global prev_time, current_pitch, current_roll
    
    try:
        ax, ay, az, gx, gy, gz = read_accel_gyro_data()
        
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        
        pitch_accel, roll_accel = calculate_pitch_roll(ax, ay, az)
        
        current_pitch = kalman_pitch.update(pitch_accel, gy, dt)
        current_roll = kalman_roll.update(roll_accel, gx, dt)

        status.update_status(key="pitch", value=current_pitch)
        status.update_status(key="roll", value=current_roll)
        
        return current_pitch, current_roll
    except Exception as e:
        print(f"Error in get_orientation: {e}")
        return current_pitch, current_roll

def initialize_sensors():
    global current_pitch, current_roll, filtered_accel_x, filtered_accel_y, filtered_accel_z
    global sensor_initialized
    
    try:
        
        if not init_mpu9250():
            print("Failed to initialize MPU9250")
            return False
        
        ax, ay, az = read_accel_data()
        pitch, roll = calculate_pitch_roll(ax, ay, az)

        kalman_pitch.angle = pitch
        kalman_roll.angle = roll

        current_pitch = pitch
        current_roll = roll
        
        filtered_accel_x = 0.0
        filtered_accel_y = 0.0
        filtered_accel_z = 0.0
        
        temp = read_temp_data()
        
        print(f"Sensors initialized. Initial values: Pitch={pitch:.2f}°, Roll={roll:.2f}°, Temp={temp:.1f}°C")
        sensor_initialized = True
        return True
    except Exception as e:
        print(f"Error initializing sensors: {e}")
        sensor_initialized = False
        return False

def read_all_sensors_and_update_status():

    if not sensor_initialized:
        if not initialize_sensors():
            print("Could not initialize sensors")
            return {}
    
    try:
        pitch, roll = get_orientation()
        
        vx, vy, vz = calculate_velocity()
        horizontal_velocity = math.sqrt(vx**2 + vy**2)
        
        temp = read_temp_data()
        
        sensor_data = {
            "pitch": pitch,
            "roll": roll,
            "velocity_x": vx,
            "velocity_y": vy,
            "velocity_z": vz,
            "horizontal_velocity": horizontal_velocity,
            "vertical_velocity": vz,
            "temp": temp
        }
        
        for key, value in sensor_data.items():
            status.update_status(key=key, value=value)
        
        return sensor_data
        
    except Exception as e:
        print(f"Error in read_all_sensors_and_update_status: {e}")
        return {}

def update_loop(update_interval=0.01):
    print("Sensor update thread started")
    try:
        while True:
            read_all_sensors_and_update_status()
            time.sleep(update_interval)
    except Exception as e:
        print(f"Sensor update thread error: {e}")
        print("Sensor update thread terminated")

def start_update():
    
    if not sensor_initialized:
        if not initialize_sensors():
            print("Failed to initialize sensors, update thread not started")
            return None
    
    update_thread = threading.Thread(target=update_loop, daemon=True)
    update_thread.start()
    return update_thread

def run_test(duration=10):
    print("Starting sensor test...")
    
    if not sensor_initialized:
        initialize_sensors()
        
    start_time = time.time()
    
    while time.time() - start_time < duration:
        data = read_all_sensors_and_update_status()
        
        if data:
            print(f"Pitch: {data['pitch']:.2f}°, Roll: {data['roll']:.2f}°, Temp: {data['temp']:.1f}°C")
            print(f"Velocity: X={data['velocity_x']:.2f} m/s, Y={data['velocity_y']:.2f} m/s, Z={data['velocity_z']:.2f} m/s")
            print(f"Horizontal Velocity: {data['horizontal_velocity']:.2f} m/s")
            print("-" * 50)
        
        time.sleep(0.1)
    
    print("Sensor test completed.")