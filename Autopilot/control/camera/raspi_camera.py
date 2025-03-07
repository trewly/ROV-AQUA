import subprocess

# Lệnh với flip camera (lật dọc và ngang)
command = "libcamera-vid -t 0 --width 1280 --height 720 --framerate 30 --codec h264 --vflip --hflip -o - | gst-launch-1.0 fdsrc ! h264parse ! rtph264pay ! udpsink host=169.254.219.222 port=5000"

print("Starting GStreamer UDP Stream with flipped camera...")

# Chạy lệnh và in stdout + stderr
process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

try:
    for line in process.stdout:
        print("[STDOUT]:", line.strip())  # In ra stdout
    for line in process.stderr:
        print("[STDERR]:", line.strip())  # In ra stderr
    process.wait()  # Chờ lệnh kết thúc
except KeyboardInterrupt:
    print("Stopping stream...")
    process.terminate()