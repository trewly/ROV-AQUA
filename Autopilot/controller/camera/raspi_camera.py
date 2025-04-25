import subprocess
import socket
import time
import os
import sys
import cv2
import threading
import signal
import atexit
from picamera2 import Picamera2, encoders #type: ignore

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Autopilot.utils.raspi_logger import LOG

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

        LOG.info("File sent successfully")
        return True
    except Exception as e:
        LOG.error(f"Error sending file: {e}")
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
            
        LOG.info(f"Image captured: {file_path}")
        return file_path
    except Exception as e:
        LOG.error(f"Error capturing image: {e}")
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

        LOG.info(f"Recording video for {duration} seconds")
        picam2.start_recording(encoder, file_path)
        time.sleep(duration)
        picam2.stop_recording()
        picam2.close()

        LOG.info(f"Video recorded: {file_path}")
        return file_path
    except Exception as e:
        LOG.error(f"Error recording video: {e}")
        return None

def record_and_send(server_ip=DEFAULT_SERVER_IP, server_port=DEFAULT_SERVER_PORT, file_path=None, duration=5):
    file_path = record_video(file_path, duration)
    if file_path:
        return send_file(server_ip, server_port, file_path)
    return False

def get_streaming_command(host=DEFAULT_SERVER_IP, port=DEFAULT_STREAM_PORT, 
                          resolution=(960, 540), framerate=30, bitrate=3000000):
    width, height = resolution
    return (f"libcamera-vid -t 0 --width {width} --height {height} "
            f"--framerate {framerate} --libav-format h264 --profile baseline "
            f"--inline --bitrate {bitrate} -o - | "
            f"gst-launch-1.0 fdsrc ! h264parse config-interval=1 ! "
            f"mpegtsmux ! udpsink host={host} port={port} sync=False async=False")

def is_network_reachable(host=DEFAULT_SERVER_IP):
    try:
        result = subprocess.run(["ping", "-c", "1", host], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
        return result.returncode == 0
    except:
        return False

def _run_streaming(command):
    global stream_process, is_streaming
    
    try:
        LOG.info("Starting video stream...")
        stream_process = subprocess.Popen(
            command, 
            shell=True,
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
        )
        
        time.sleep(0.5)
        if stream_process.poll() is not None:
            error_output = stream_process.stderr.read().decode('utf-8')
            LOG.error(f"Stream process error: {error_output}")
            is_streaming = False
            return
            
        is_streaming = True
        LOG.info("Stream started successfully")
        
        stream_process.wait()
        
    except Exception as e:
        LOG.error(f"Error during streaming: {e}")
    finally:
        LOG.info("Stream process terminated")
        is_streaming = False
        stream_process = None

def start_stream(host=DEFAULT_SERVER_IP, port=DEFAULT_STREAM_PORT, 
                resolution=(960, 540), framerate=30, bitrate=3000000):
    global stream_thread, is_streaming, stream_process
    
    retry_count = 0
    max_retries = 5
    while retry_count < max_retries:
        if is_network_reachable(host):
            break
        LOG.info(f"Waiting for network connection to {host}... attempt {retry_count+1}/{max_retries}")
        time.sleep(1)
        retry_count += 1
    
    if retry_count == max_retries:
        LOG.error(f"Network destination {host} is unreachable after {max_retries} attempts")
        return False
    
    with stream_lock:
        if is_streaming and stream_process and stream_process.poll() is None:
            LOG.info("Stream is already active")
            return True
        
        is_streaming = False
        if stream_process is not None:
            try:
                stop_stream()
            except:
                pass
            
        command = get_streaming_command(host, port, resolution, framerate, bitrate)
        
        stream_thread = threading.Thread(
            target=_run_streaming, 
            args=(command,),
            daemon=True
        )
        stream_thread.start()
        
        wait_time = 0
        max_wait = 5
        while wait_time < max_wait and not is_streaming:
            time.sleep(0.5)
            wait_time += 0.5
            
            if stream_process and stream_process.poll() is not None:
                LOG.error("Stream process terminated unexpectedly")
                return False
        
        if not is_streaming:
            LOG.error("Failed to start stream after waiting")
            try:
                if stream_process:
                    os.killpg(os.getpgid(stream_process.pid), signal.SIGINT)
                    time.sleep(0.5)
                    if stream_process.poll() is None:
                        os.killpg(os.getpgid(stream_process.pid), signal.SIGTERM)
            except Exception as e:
                LOG.error(f"Error terminating failed stream: {e}")
            return False
        
        return True

def stop_stream():
    global stream_process, is_streaming

    with stream_lock:
        if not is_streaming or stream_process is None:
            LOG.info("Stream is not active")
            return False

        try:
            LOG.info("Stopping stream gracefully...")

            if stream_process is not None:
                os.killpg(os.getpgid(stream_process.pid), signal.SIGINT)

                for _ in range(20):
                    if stream_process.poll() is not None:
                        break
                    time.sleep(0.1)

                if stream_process and stream_process.poll() is None:
                    LOG.info("Process still running, sending SIGTERM...")
                    os.killpg(os.getpgid(stream_process.pid), signal.SIGTERM)

                    for _ in range(30):
                        if stream_process.poll() is not None:
                            break
                        time.sleep(0.1)

                    if stream_process and stream_process.poll() is None:
                        LOG.info("Process still running, forcing SIGKILL...")
                        os.killpg(os.getpgid(stream_process.pid), signal.SIGKILL)

            LOG.info("Stream stopped successfully")
            return True
        except Exception as e:
            LOG.error(f"Error stopping stream: {e}")
            try:
                if stream_process and stream_process.poll() is None:
                    stream_process.terminate()
                    time.sleep(0.5)
                    if stream_process.poll() is None:
                        stream_process.kill()
            except Exception as kill_error:
                LOG.error(f"Failed to kill process: {kill_error}")
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
    stop_stream()
    if stream_thread and stream_thread.is_alive():
        stream_thread.join(timeout=1)
        if stream_thread.is_alive():
            LOG.warning("Stream thread did not terminate in time")
            stream_thread = None

atexit.register(cleanup)

if __name__ == "__main__":
    try:
        success = start_stream()
        if success:
            while True:
                pass
    except KeyboardInterrupt:
        stop_stream()
    finally:
        stop_stream()