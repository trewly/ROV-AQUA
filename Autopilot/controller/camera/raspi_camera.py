import subprocess
import socket
import time
import os
import cv2
import threading
import signal
from picamera2 import Picamera2, encoders #type: ignore

DEFAULT_SERVER_IP = "169.254.54.121"
DEFAULT_SERVER_PORT = 5001
DEFAULT_STREAM_PORT = 5000
DEFAULT_RESOLUTION = (1920, 1080)
DEFAULT_FRAMERATE = 30
DEFAULT_BITRATE = 3000000
DEFAULT_OUTPUT_DIR = "/tmp"

stream_process = None
stream_thread = None
is_streaming = False
stream_lock = threading.Lock()

def send_file(server_ip, server_port, file_path):
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(10)
        client_socket.connect((server_ip, server_port))

        file_name = os.path.basename(file_path) + "\n"
        client_socket.sendall(file_name.encode("utf-8"))

        with open(file_path, "rb") as file:
            while chunk := file.read(4096):
                client_socket.sendall(chunk)

        print(f"Đã gửi file: {file_path}")
        return True
    except Exception as e:
        print(f"Lỗi khi gửi file: {e}")
        return False
    finally:
        try:
            client_socket.close()
        except:
            pass

def capture_image(file_path=None, rotate=True, resolution=DEFAULT_RESOLUTION):
    if file_path is None:
        file_path = os.path.join(DEFAULT_OUTPUT_DIR, f"image_{int(time.time())}.jpg")
    
    try:
        picam2 = Picamera2()
        config = picam2.create_still_configuration(main={"size": resolution})
        picam2.configure(config)

        picam2.start()
        time.sleep(2)

        picam2.capture_file(file_path)
        picam2.close()
        
        if rotate:
            image = cv2.imread(file_path)
            rotated_image = cv2.rotate(image, cv2.ROTATE_180)
            cv2.imwrite(file_path, rotated_image)
            
        print(f"Đã chụp ảnh: {file_path}")
        return file_path
    except Exception as e:
        print(f"Lỗi khi chụp ảnh: {e}")
        return None

def capture_and_send(server_ip=DEFAULT_SERVER_IP, server_port=DEFAULT_SERVER_PORT, file_path=None, rotate=True):
    file_path = capture_image(file_path, rotate)
    if file_path:
        return send_file(server_ip, server_port, file_path)
    return False

def record_video(file_path=None, duration=5, resolution=DEFAULT_RESOLUTION, bitrate=10000000):
    if file_path is None:
        file_path = os.path.join(DEFAULT_OUTPUT_DIR, f"video_{int(time.time())}.h264")
    
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": resolution})
        picam2.configure(config)

        encoder = encoders.H264Encoder(bitrate=bitrate)

        print(f"Đang quay video ({duration} giây)...")
        picam2.start_recording(encoder, file_path)
        time.sleep(duration)
        picam2.stop_recording()
        picam2.close()

        print(f"Đã quay xong video: {file_path}")
        return file_path
    except Exception as e:
        print(f"Lỗi khi quay video: {e}")
        return None

def record_and_send(server_ip=DEFAULT_SERVER_IP, server_port=DEFAULT_SERVER_PORT, file_path=None, duration=5):
    file_path = record_video(file_path, duration)
    if file_path:
        return send_file(server_ip, server_port, file_path)
    return False

command = ("libcamera-vid -t 0 --width 960 --height 540 --framerate 30 --libav-format h264 --profile baseline --inline --bitrate 3000000 -o - | gst-launch-1.0 fdsrc ! h264parse config-interval = 1 ! mpegtsmux ! udpsink host=169.254.54.121 port=5000 sync=False async=False")

def is_network_reachable(host=DEFAULT_SERVER_IP):
    try:
        result = subprocess.run(["ping", "-c", "1", host], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
        return result.returncode == 0
    except:
        return False

def _run_streaming(command):
    global stream_process, is_streaming
    
    try:
        print(f"Starting stream with command: {command}")
        stream_process = subprocess.Popen(
            command, 
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
        )
        
        time.sleep(0.5)
        if stream_process.poll() is not None:
            error_output = stream_process.stderr.read().decode('utf-8')
            print(f"Stream failed to start. Error: {error_output}")
            is_streaming = False
            return
            
        is_streaming = True
        print("Stream started successfully")
        
        return_code = stream_process.wait()
        print(f"Stream process exited with code: {return_code}")
        
    except Exception as e:
        print(f"Streaming error: {e}")
    finally:
        is_streaming = False
        print("Stream terminated")

def start_stream(host=DEFAULT_SERVER_IP, port=DEFAULT_STREAM_PORT, 
                resolution=DEFAULT_RESOLUTION, framerate=DEFAULT_FRAMERATE, 
                bitrate=DEFAULT_BITRATE):
    global stream_thread, is_streaming
    while True:
        if is_network_reachable(host):
            break
        time.sleep(0.1)
    
    with stream_lock:
        if is_streaming:
            print("Stream already running")
            return True
            
        is_streaming = False
            
        width, height = resolution
        
        stream_thread = threading.Thread(
            target=_run_streaming, 
            args=(command,),
            daemon=True
        )
        stream_thread.start()
        time.sleep(2)
        
        if not is_streaming:
            print("Failed to start stream")
            return False
            
        print(f"Stream started successfully to {host}:{port}")
        return True

def stop_stream():
    global stream_process, is_streaming
    
    with stream_lock:
        if not is_streaming or stream_process is None:
            print("No active stream to stop")
            return False
            
        try:
            print("Stopping stream...")
            
            os.killpg(os.getpgid(stream_process.pid), signal.SIGTERM)
            
            for _ in range(30):
                if stream_process.poll() is not None:
                    break
                time.sleep(0.1)
                
            if stream_process.poll() is None:
                print("Force killing stream process...")
                os.killpg(os.getpgid(stream_process.pid), signal.SIGKILL)
                
            print("Stream stopped successfully")
            return True
        except Exception as e:
            print(f"Error stopping stream: {e}")
            try:
                os.killpg(os.getpgid(stream_process.pid), signal.SIGKILL)
            except:
                pass
            return False
        finally:
            is_streaming = False
            stream_process = None

def is_stream_active():
    with stream_lock:
        if is_streaming and stream_process is not None:
            if stream_process.poll() is None:
                return True
            else:
                is_streaming = False
                return False
        return False

def restart_stream(host=DEFAULT_SERVER_IP, port=DEFAULT_STREAM_PORT, 
                 resolution=DEFAULT_RESOLUTION, framerate=DEFAULT_FRAMERATE, 
                 bitrate=DEFAULT_BITRATE):
    stop_stream()
    time.sleep(1)
    return start_stream(host, port, resolution, framerate, bitrate)

def cleanup():
    if is_streaming:
        stop_stream()

if __name__ == "__main__":
    try:
        print("Starting video stream...")
        success = start_stream()
        
        if success:
            print("Stream started successfully")
            print("Press Ctrl+C to stop streaming")
            while True:
                pass
        else:
            print("Failed to start stream")
            
    except KeyboardInterrupt:
        print("\nStopping streaming...")
    finally:
        cleanup()