import subprocess
import argparse
import threading
import socket
import time

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

tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('', 43723))


def recv_udp():
    while True:
        data, addr = udp_sock.recvfrom(1024)
        tcp_sock.sendall(data)


def recv_tcp():
    while True:
        data = tcp_sock.recv(1024)
        if len(data) == 0:
            raise Exception("TCP socket has closed")
        udp_sock.sendto(data, ("127.0.0.1", 43721))


def init_tcp():
    time.sleep(3)
    tcp_sock.connect(("127.0.0.1", 43722))
    threading.Thread(target=recv_udp).start()
    threading.Thread(target=recv_tcp).start()


threading.Thread(target=init_tcp).start()


while True:
    subprocess.run([
        "ssh",
        f"{args.username}@{args.hostname}",
        "-p", args.port,
        "-i", args.keyfile,
        "-T",
        "-L", f"43722:/home/{args.username}/lunadev-2024/lunadev/tele.sock"
    ])
