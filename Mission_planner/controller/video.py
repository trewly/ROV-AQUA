import subprocess

port = "5000"

gst_command = (
    f"gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H264 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! d3dvideosink"
)

print(f"Receiving stream on PC from Raspberry Pi on port {port}...")
subprocess.run(gst_command, shell=True)
