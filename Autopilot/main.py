from Autopilot.communication import raspi_mavlink as mav
from Autopilot.controller.motor import raspi_motor_control as rov
from Autopilot.system_info.status import raspi_status as status
from Autopilot.controller.camera import raspi_camera as camera
from Autopilot.system_info.sensor import raspi_sensor_read as sensor

sensor.initialize_sensors()
sensor.start_update()
camera.start_stream()
mav.init_mavlink()
rov.initialize_motors()
status.init_status()

while True:
    pass