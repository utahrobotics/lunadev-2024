import subprocess
import sys
import os
from time import sleep


def main():
    full_sock_path = "/root/lunadev-2024/lunadev/tele.sock"
    if os.path.exists(full_sock_path):
        os.remove(full_sock_path)

    popen = subprocess.Popen(
        [
            "socat",
            "UNIX-LISTEN:lunadev/tele.sock,reuseaddr,fork,",
            "UDP4-LISTEN:43721"
        ],
        stderr=subprocess.DEVNULL
    )

    while not os.path.exists(full_sock_path):
        sleep(0.3)

    subprocess.run(["chmod", "777", full_sock_path])
    print("Telemetry tunnel started successfully!")
    sys.exit(popen.wait())


if __name__ == "__main__":
    main()
