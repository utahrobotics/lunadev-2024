import subprocess
import argparse
import threading
import socket
import sys
import time

parser = argparse.ArgumentParser(
    prog='VNC Tunnel',
    description="Creates an encrypted connection to your container's"
    "VNC Server"
)

parser.add_argument('username')
# parser.add_argument('hostname')
# parser.add_argument('port')
parser.add_argument('keyfile')
parser.add_argument('-tp', '--tunnel_port', default=43721, type=int)
parser.add_argument('-d', "--diagnostics", action="store_true")

args = parser.parse_args()

UDP_HEADER_SIZE = 8     # 8 bytes
DIAGNOSTIC_WINDOW = 3
DIAGNOSTIC_DELAY = 2

# First item in list is timestamp
# Second item is packet size
diagnostic_buffer = []
diagnostics_enabled = args.diagnostics
running = True


def print_diagnostics():
    while running:
        time.sleep(DIAGNOSTIC_DELAY)

        for i, (timestamp, _) in enumerate(diagnostic_buffer):
            if time.time() - timestamp < DIAGNOSTIC_WINDOW:
                del diagnostic_buffer[0:i]
                break
        else:
            diagnostic_buffer.clear()

        print(
            "Bandwidth:",
            round(sum((n for _, n in diagnostic_buffer)) / 1000 / DIAGNOSTIC_WINDOW * 8, 3),
            "kb/s"
        )


if diagnostics_enabled:
    threading.Thread(target=print_diagnostics).start()


def track_data(data: bytes):
    if not diagnostics_enabled:
        return

    diagnostic_buffer.append((
        time.time(),
        len(data) + UDP_HEADER_SIZE
    ))


tcp_sock = None
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('', 43723))
udp_sock.setblocking(False)
connected = False


def recv_udp():
    while running:
        if not connected:
            time.sleep(0.3)
            continue

        try:
            data, _ = udp_sock.recvfrom(1024)
            track_data(data)
            tcp_sock.sendall(data)
        except ConnectionResetError:
            pass
        except BlockingIOError:
            time.sleep(0.05)


process = None
server_port = args.tunnel_port


def recv_tcp():
    global running, connected
    while True:
        try:
            data = tcp_sock.recv(1024)
            if len(data) == 0:
                break
            track_data(data)
            udp_sock.sendto(data, ("127.0.0.1", server_port))
        except ConnectionResetError:
            break

    connected = False
    process.kill()
    print("Tunnel has closed!")


threading.Thread(target=recv_udp).start()


while True:
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    process = subprocess.Popen([
        "ssh",
        f"{args.username}@5.tcp.ngrok.io",
        "-p", "22735",
        "-i", args.keyfile,
        "-T",
        "-L", f"43722:/home/{args.username}/lunadev-2024/lunadev/tele.sock"
    ])

    time.sleep(2)

    if process.returncode is not None:
        sys.exit(process.returncode)

    tcp_sock.connect(("127.0.0.1", 43722))
    connected = True
    threading.Thread(target=recv_tcp).start()

    try:
        process.wait()
        time.sleep(2)
    except KeyboardInterrupt:
        running = False
        break