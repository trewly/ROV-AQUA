import subprocess
import socket
import time
import cv2
import threading
from picamera2 import Picamera2, encoders # type: ignore

def send_file(server_ip, server_port, file_path):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))

    file_name = file_path.split("/")[-1] + "\n"
    client_socket.sendall(file_name.encode("utf-8"))

    with open(file_path, "rb") as file:
        while chunk := file.read(4096):
            client_socket.sendall(chunk)

    print(f"Đã gửi file: {file_path}")
    client_socket.close()

def run_streaming():
    command = "libcamera-vid -t 0 --width 1920 --height 1080 --framerate 30 --codec h264 --bitrate 3000000 -o - | gst-launch-1.0 fdsrc ! h264parse config-interval = 10 ! mpegtsmux ! udpsink host=169.254.54.121 port=5000 sync=false async=false"
    global process
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    process.wait()
thread = threading.Thread(target=run_streaming, daemon=True)
thread.start()
thread.join()

def stop_video_stream():
    print("Stopping GStreamer UDP Stream...")
    process.terminate()

def capture_and_send(server_ip, server_port, file_path="image.jpg"):
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (1920, 1080)})
    picam2.configure(config)

    picam2.start()
    time.sleep(2)

    picam2.capture_file(file_path)
    print(f"Đã chụp ảnh gốc: {file_path}")

    image = cv2.imread(file_path)
    rotated_image = cv2.rotate(image, cv2.ROTATE_180)

    cv2.imwrite(file_path, rotated_image)
    print(f"Ảnh đã xoay 180°: {file_path}")

    send_file(server_ip, server_port, file_path)
#capture_and_send(server_ip="169.254.54.121", server_port=5001)

def record_and_send(server_ip, server_port, file_path="video.h264", duration=5):
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (1920, 1080)})
    picam2.configure(config)

    encoder = encoders.H264Encoder(bitrate=10000000)

    print(f"Đang quay video ({duration} giây)...")
    picam2.start_recording(encoder, file_path)
    time.sleep(duration)
    picam2.stop_recording()

    print(f"Đã quay xong video: {file_path}")
    send_file(server_ip, server_port, file_path)
#record_and_send(server_ip="169.254.54.121", server_port=5001, duration=10)
