import rclpy
from threading import Thread
from rclpy.node import Node
from positioning.pozyx_localizer import PozyxLocalizer
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Header


class Positioning(Node):
    def __init__(self):
        super().__init__("positioning")
        self.localizer = PozyxLocalizer("PozyxConfig.yaml")
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/pozyx/pose',
            10
        )

        self.thr = Thread(target=self.run)
        self.thr.start()

    def run(self):
        while True:
            try:
                self.localizer.loop()
            except KeyboardInterrupt:
                return
            try:
                self.pose_pub.publish(
                    PoseWithCovarianceStamped(
                        header=Header(
                            stamp=self.get_clock().now().to_msg(),
                            frame_id="pozyx_frame"
                        ),
                        pose=PoseWithCovariance(pose=self.localizer.pose)
                    )
                )
            except Exception:
                return


def main(args=None):
    rclpy.init(args=args)

    node = Positioning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
