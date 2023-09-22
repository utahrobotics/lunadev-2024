import rclpy
from rclpy.node import Node
import subprocess
from sensor_msgs.msg import Image
from global_msgs.msg import CompressedImagePacket
from threading import Thread
from time import time


class CameraCompress(Node):
    def __init__(self):
        super().__init__("camera_compress")

        self.target_fps = 4
        self.ffmpeg = subprocess.Popen(
            [
                "ffmpeg",
                "-f", "rawvideo",
                "-pix_fmt", "rgb24",
                "-s", "1280x720",
                "-r", "4",
                "-i", "-",
                "-c:v", "libvpx-vp9",
                "-b:v", "1500k",
                "-vf", 'scale=64:-1',
                "-f", "mpegts", "-"
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self.camera_listener = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback_callback,
            30
        )
        self.compressed_pub = self.create_publisher(CompressedImagePacket, 'compressed_image', 100)

        self.count = 0
        self.probe_complete = False
        self.last_processed_frame_time = 0

        self.thr = Thread(target=self.read_stdout)
        self.thr.start()

    def read_stdout(self):
        try:
            buffer = bytearray()
            while True:
                try:
                    data = self.ffmpeg.stdout.read(188)
                except KeyboardInterrupt:
                    break

                if len(data) == 0:
                    err = b""
                    while True:
                        packet = self.ffmpeg.stderr.read(1024)
                        if len(packet) == 0:
                            break
                        err += packet
                    raise Exception(err.decode())

                buffer += data
                self.probe_complete = True
                while len(buffer) >= 188:
                    self.compressed_pub.publish(
                        CompressedImagePacket(data=buffer[0:188])
                    )
                    del buffer[0:188]

        except BrokenPipeError:
            # Error will be caught in camera callback
            return

    def camera_callback_callback(self, img: Image):
        if self.probe_complete and time() - self.last_processed_frame_time < 1.0 / self.target_fps:
            return

        self.last_processed_frame_time = time()
        try:
            i = 0
            while i < len(img.data):
                i += self.ffmpeg.stdin.write(bytes(img.data[i::]))

        except BrokenPipeError:
            err = b""
            while True:
                packet = self.ffmpeg.stderr.read(1024)
                if len(packet) == 0:
                    break
                err += packet
            raise Exception(err.decode())

        except KeyboardInterrupt:
            return


def main(args=None):
    rclpy.init(args=args)

    node = CameraCompress()

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
