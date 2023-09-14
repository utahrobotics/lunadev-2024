import rclpy
from threading import Thread
from rclpy.node import Node
from positioning.pozyx_localizer import PozyxLocalizer


class Positioning(Node):
    def __init__(self):
        super().__init__("positioning")
        self.localizer = PozyxLocalizer("PozyxConfig.yaml")

        self.thr = Thread(target=self.run)
        self.thr.start()

    def run(self):
        while True:
            self.localizer.loop()
            print(self.localizer.pose)


def main(args=None):
    rclpy.init(args=args)

    node = Positioning()

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
