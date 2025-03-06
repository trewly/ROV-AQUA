import smbus2 as smbus
import time
import math

G = 9.80665

MPU9250_ADDR = 0x68
AK09911C_ADDR = 0x0D

REG_WIA = 0x01
REG_ST1 = 0x10
REG_ST2 = 0x18
REG_CNTL2 = 0x31

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

MAG_XOUT_L = 0x11
MAG_XOUT_H = 0x12
MAG_YOUT_L = 0x13
MAG_YOUT_H = 0x14
MAG_ZOUT_L = 0x15
MAG_ZOUT_H = 0x16

XA_OFFSET_H = 0x77
XA_OFFSET_L = 0x78
YA_OFFSET_H = 0x7A
YA_OFFSET_L = 0x7B
ZA_OFFSET_H = 0x7D
ZA_OFFSET_L = 0x7E

bus = smbus.SMBus(1)

def read_word_mpu(register):
    high = bus.read_byte_data(MPU9250_ADDR, register)
    low = bus.read_byte_data(MPU9250_ADDR, register + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value -= 0x10000
    return value

def read_word_ak(register):
    low = bus.read_byte_data(AK09911C_ADDR, register)
    bus.read_byte_data(AK09911C_ADDR, REG_ST2)
    high = bus.read_byte_data(AK09911C_ADDR, register + 1)
    bus.read_byte_data(AK09911C_ADDR, REG_ST2)
    value = (high << 8) | low
    if value > 0x8000:
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
    mag_x = read_word_ak(MAG_XOUT_L)
    mag_y = read_word_ak(MAG_YOUT_L)
    mag_z = read_word_ak(MAG_ZOUT_L)
    return mag_x, mag_y, mag_z

def read_mag_data_calibrated(calib_x, calib_y, calib_z):
    mag_x, mag_y, mag_z = read_mag_data()
    mag_x -= calib_x
    mag_y -= calib_y
    mag_z -= calib_z
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