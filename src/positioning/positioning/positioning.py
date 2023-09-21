import rclpy
from threading import Thread
from rclpy.node import Node
from positioning.pozyx_localizer import PozyxLocalizer
from geometry_msgs.msg import Pose, PoseWithCovariance


class Positioning(Node):
    def __init__(self):
        super().__init__("positioning")
        self.localizer = PozyxLocalizer("PozyxConfig.yaml")
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovariance,
            'pozyx_pose',
            10
        )
        self.pose_pub = self.create_publisher(
            Pose,
            'pozyx_pose_no_cov',
            10
        )

        self.thr = Thread(target=self.run)
        self.thr.start()

    def run(self):
        while True:
            self.localizer.loop()
            self.pose_cov_pub.publish(
                PoseWithCovariance(pose=self.localizer.pose)
            )
            self.pose_pub.publish(
                self.localizer.pose
            )


def main(args=None):
    rclpy.init(args=args)

    node = Positioning()

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
