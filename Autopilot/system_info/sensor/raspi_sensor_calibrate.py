import time
import math
import numpy as np

import Autopilot.system_info.sensor.raspi_sensor_read as sensor

def calibrate_mag(sample_count=1500):
    print("Di chuyển cảm biến theo hình số 8 để hiệu chuẩn...")
    time.sleep(2)

    mag_min = np.array([999999, 999999, 999999], dtype=float)
    mag_max = np.array([-999999, -999999, -999999], dtype=float)

    for i in range(sample_count):
        status = sensor.read_mag_status()
        if status & 0x01:
            mag = sensor.read_mag_data()
            mag_min = np.minimum(mag_min, mag)
            mag_max = np.maximum(mag_max, mag)
        time.sleep(0.013)

    print("Hoàn tất hiệu chuẩn!")

    m_bias = (mag_max + mag_min) / 2
    print(f"Hard Iron Bias: {m_bias}")

    scale_factors = (mag_max - mag_min) / 2
    avg_radius = np.mean(scale_factors)
    m_scale = avg_radius / scale_factors
    print(f"Soft Iron Scale: {m_scale}")

    return m_bias, m_scale

# m_bias, m_scale = calibrate_mag()

# while True:
#     status = sensor.read_mag_status()
#     if status & 0x01:  # Kiểm tra dữ liệu mới
#         mag = sensor.read_mag_data()
#         mag_corrected = (mag - m_bias) * m_scale  # Áp dụng hiệu chỉnh
#         angle_xy = math.atan2(mag_corrected[1], mag_corrected[0])
#         if angle_xy < 0:
#             angle_xy += 2 * math.pi
#         angle_xy = 360 - math.degrees(angle_xy)
#         print(f"Angle XY: {angle_xy:.2f}°")
#     else:
#         print("Chưa có dữ liệu mới, chờ thêm...")
#     time.sleep(0.013)  # 75Hz