import subprocess
import sys
import os
from time import sleep


parent_dir = os.path.dirname(__file__)
full_sock_path = f"{parent_dir}/tele.sock"
if os.path.exists(full_sock_path):
    os.remove(full_sock_path)

popen = subprocess.Popen(
    ["socat", "UNIX-LISTEN:lunadev/tele.sock,reuseaddr,", "UDP4-LISTEN:43721"]
)

while not os.path.exists(full_sock_path):
    sleep(0.3)

subprocess.run(["chmod", "777", full_sock_path])
sys.exit(popen.wait())
