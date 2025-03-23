import smbus2 as smbus
import time
import math
import numpy as np

from Autopilot.system_info.status import raspi_status as status

G = 9.80665

MPU9250_ADDR = 0x68
HMC5883L_ADDR = 0x1E

MAG_CONFIG_A = 0x00
MAG_CONFIG_B = 0x01
MAG_MODE = 0x02
STATUS_REG = 0x09

MAG_MODE_CONTINUOUS = 0x00

TEMP_OUT = 0x41

ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

MAG_XOUT_L = 0x03
MAG_XOUT_H = 0x04
MAG_YOUT_L = 0x05
MAG_YOUT_H = 0x06
MAG_ZOUT_L = 0x07
MAG_ZOUT_H = 0x08

XA_OFFSET_H = 0x77
XA_OFFSET_L = 0x78
YA_OFFSET_H = 0x7A
YA_OFFSET_L = 0x7B
ZA_OFFSET_H = 0x7D
ZA_OFFSET_L = 0x7E

bus = smbus.SMBus(1)

def mag_init():
    bus.write_byte_data(HMC5883L_ADDR, MAG_CONFIG_A, 0x78)
    bus.write_byte_data(HMC5883L_ADDR, MAG_CONFIG_B, 0x20)
    bus.write_byte_data(HMC5883L_ADDR, MAG_MODE, MAG_MODE_CONTINUOUS)

def read_mag_status():
    bus.write_byte(HMC5883L_ADDR, STATUS_REG)
    return bus.read_byte(HMC5883L_ADDR)

def read_word_mpu(register):
    high = bus.read_byte_data(MPU9250_ADDR, register)
    low = bus.read_byte_data(MPU9250_ADDR, register + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value -= 0x10000
    return value

def read_temp_data():
    temp = read_word_mpu(TEMP_OUT)
    temp = (temp / 340) + 36.53
    return temp

def read_gyro_data():
    gyro_x = read_word_mpu(GYRO_XOUT_H)
    gyro_y = read_word_mpu(GYRO_YOUT_H)
    gyro_z = read_word_mpu(GYRO_ZOUT_H)
    return gyro_x, gyro_y, gyro_z

def read_accel_data():
    accel_x = read_word_mpu(ACCEL_XOUT_H)
    accel_y = read_word_mpu(ACCEL_YOUT_H)
    accel_z = read_word_mpu(ACCEL_ZOUT_H)
    return accel_x, accel_y, accel_z

def read_mag_data():
    bus.write_byte(HMC5883L_ADDR, MAG_XOUT_L)
    data = bus.read_i2c_block_data(HMC5883L_ADDR, MAG_XOUT_L, 6)

    x = (data[0] << 8) | data[1]
    z = (data[2] << 8) | data[3]
    y = (data[4] << 8) | data[5]

    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z

    return np.array([x, y, z], dtype=float)

def read_mag_data_calibrated():
    mag_x, mag_y, mag_z = read_mag_data()
    mag_x -= status.read_status("mag_x")
    mag_y -= status.read_status("mag_y")
    mag_z -= status.read_status("mag_z")
    mag_x = mag_x * status.read_status("mag_scale_x")
    mag_y = mag_y * status.read_status("mag_scale_y")
    mag_z = mag_z * status.read_status("mag_scale_z")
    return mag_x, mag_y, mag_z

def read_angle_xy(x, y):
    angle = math.atan2(y, x)
    if angle < 0:
        angle += 2*math.pi
    return angle

def read_angle_xz(x, z):
    angle = math.atan2(z, x)
    if angle < 0:
        angle += 2*math.pi
    return angle

def read_angle_yz(y, z):
    angle = math.atan2(z, y)
    if angle < 0:
        angle += 2*math.pi
    return angle

def read_3d_gravity(angle_xz, angle_yz):
    gravity_x = G * math.sin(angle_xz)
    gravity_y = -G * math.cos(angle_xz) * math.sin(angle_yz)
    gravity_z = G * math.cos(angle_xz) * math.cos(angle_yz)
    return gravity_x, gravity_y, gravity_z

def read_accel_data_gravity_calibrated():
    accel_x, accel_y, accel_z = read_accel_data()

    angle_xz = read_angle_xz(accel_x, accel_z)
    angle_yz = read_angle_yz(accel_y, accel_z)

    gravity_x, gravity_y, gravity_z = read_3d_gravity(angle_xz, angle_yz)

    accel_x -= gravity_x
    accel_y -= gravity_y
    accel_z -= gravity_z
    return accel_x, accel_y, accel_z

def read_gyro_data_dps():
    gyro_x, gyro_y, gyro_z = read_gyro_data()
    gyro_x = (gyro_x / 32768) * 250
    gyro_y = (gyro_y / 32768) * 250
    gyro_z = (gyro_z / 32768) * 250
    return gyro_x, gyro_y, gyro_z
# bus.write_byte_data(AK09911C_ADDR, REG_CNTL2, 0x08)
# time.sleep(0.1)
# calib_x, calib_y, calib_z = calibrate.calibrate_mag()