import subprocess
import argparse

parser = argparse.ArgumentParser(
    prog='VNC Tunnel',
    description="Creates an encrypted connection to your container's"
    "VNC Server"
)

parser.add_argument('username')
parser.add_argument('hostname')
parser.add_argument('port')
parser.add_argument('keyfile')

args = parser.parse_args()

while True:
    subprocess.run([
        "ssh",
        f"{args.username}@{args.hostname}",
        "-p", args.port,
        "-i", args.keyfile,
        "-T",
        "-L", f"5901:/home/{args.username}/lunadev-2024/lunadev/vnc.sock"
    ])
