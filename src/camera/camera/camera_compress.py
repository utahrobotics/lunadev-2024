import rclpy
from rclpy.node import Node
from PIL import Image
from sensor_msgs.msg import Image as ImageMsg
from global_msgs.msg import CompressedImagePacket
from time import time
import io


class CameraCompress(Node):
    def __init__(self):
        super().__init__("camera_compress")

        self.target_fps = 4

        self.camera_listener = self.create_subscription(
            ImageMsg,
            '/camera/color/image_raw',
            self.camera_callback_callback,
            30
        )
        self.compressed_pub = self.create_publisher(
            CompressedImagePacket,
            'compressed_image',
            100
        )

        self.count = 0
        self.last_processed_frame_time = 0

    def camera_callback_callback(self, img: ImageMsg):
        if time() - self.last_processed_frame_time < 1.0 / self.target_fps:
            return

        self.last_processed_frame_time = time()
        img = Image.frombytes("RGB", (1280, 720), bytes(img.data))  \
            .resize((int(1280 * 0.15), int(720 * 0.15)), Image.LANCZOS)

        webp_buf = io.BytesIO()
        img.save(webp_buf, "webp", quality=0, method=6)
        self.compressed_pub.publish(
            CompressedImagePacket(data=webp_buf.getbuffer())
        )


def main(args=None):
    rclpy.init(args=args)

    node = CameraCompress()

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
