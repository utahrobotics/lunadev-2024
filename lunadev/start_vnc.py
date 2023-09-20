#!/usr/bin/python3
import subprocess
import secrets
import os
from time import sleep


parent_dir = os.path.dirname(__file__)
full_sock_path = f"{parent_dir}/vnc.sock"
if os.path.exists(full_sock_path):
    os.remove(full_sock_path)

subprocess.Popen(
    ["socat", "UNIX-LISTEN:lunadev/vnc.sock,reuseaddr,", "TCP:127.0.0.1:5900"]
)

while not os.path.exists(full_sock_path):
    sleep(0.3)

subprocess.run(["chmod", "777", full_sock_path])

password = secrets.token_urlsafe(32)
print(f"VNC Password: {password}")

with open(f"{os.path.dirname(__file__)}/vnc_logs.txt", "w") as log_file:
    result = subprocess.run(
        ["x11vnc", "-passwd", password, "-forever", "-create"],
        stderr=log_file,
        stdout=log_file
    )

if result.returncode != 0:
    print("VNC Server faced an error, refer to vnc_logs.txt")
