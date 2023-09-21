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

        self.target_fps = 10
        self.ffmpeg = subprocess.Popen(
            [
                "ffmpeg",
                "-f", "rawvideo",
                "-pix_fmt", "rgb24",
                "-s", "1280x720",
                "-r", "10",
                "-i", "-",
                "-c:v", "libvpx-vp9",
                "-b:v", "1500k",
                # "-g", "72",
                "-f", "matroska", "-"
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
            while True:
                data = self.ffmpeg.stdout.read(1024 * 8)

                if len(data) == 0:
                    err = b""
                    while True:
                        packet = self.ffmpeg.stderr.read(1024)
                        if len(packet) == 0:
                            break
                        err += packet
                    raise Exception(err.decode())

                self.compressed_pub.publish(CompressedImagePacket(data=data))
                self.probe_complete = True

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


def main(args=None):
    rclpy.init(args=args)

    node = CameraCompress()

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
