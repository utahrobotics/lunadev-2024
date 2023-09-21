import subprocess
import sys
import os
from time import sleep


def main():
    full_sock_path = "/root/lunadev-2024/lunadev/tele.sock"
    if os.path.exists(full_sock_path):
        os.remove(full_sock_path)

    popen = subprocess.Popen(
        ["socat", "UNIX-LISTEN:lunadev/tele.sock,reuseaddr,", "UDP4-LISTEN:43721"]
    )

    while not os.path.exists(full_sock_path):
        sleep(0.3)

    subprocess.run(["chmod", "777", full_sock_path])
    if __name__ == "__main__":
        print("Telemetry tunnel started successfully!")
        sys.exit(popen.wait())
    else:
        print("[telemetry_tunnel] Telemetry tunnel started successfully!")


if __name__ == "__main__":
    main()
