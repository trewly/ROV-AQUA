import socket
import subprocess

def receive_file(server_ip, server_port, save_path):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(1)
    print("Đang chờ kết nối từ Raspberry Pi...")

    conn, addr = server_socket.accept()
    print(f"Kết nối từ: {addr}")

    file_name = b""
    while not file_name.endswith(b"\n"):
        file_name += conn.recv(1)
    file_name = file_name.decode().strip()
    
    save_path = file_name if save_path is None else save_path
    with open(save_path, "wb") as file:
        while chunk := conn.recv(4096):
            file.write(chunk)

    print(f"Đã nhận file và lưu tại: {save_path}")
    conn.close()
    server_socket.close()

receive_file(server_ip="0.0.0.0", server_port=5001, save_path="Mission_planner\status\status.json")

def receive_video(server_ip, server_port, save_path):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(1)
    print("Đang chờ kết nối từ Raspberry Pi...")

    conn, addr = server_socket.accept()
    print(f"Kết nối từ: {addr}")

    with open(save_path, "wb") as file:
        while chunk := conn.recv(4096):
            file.write(chunk)

    print(f"Đã nhận video và lưu tại: {save_path}")
    conn.close()
    server_socket.close()

    mp4_path = save_path.replace(".h264", ".mp4")
    cmd = f'ffmpeg -y -framerate 30 -i "{save_path}" -c copy "{mp4_path}"'
    subprocess.run(cmd, shell=True, check=True)

    print(f"Đã chuyển đổi thành: {mp4_path}")

#receive_video(server_ip="0.0.0.0", server_port=5001, save_path="received_video.h264")