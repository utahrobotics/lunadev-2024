from typing import List
import rclpy
from threading import Thread
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Pose, Point, Quaternion as MsgQuaternion
from std_msgs.msg import Header
from time import sleep

from pypozyx import (Coordinates, POZYX_SUCCESS, PozyxConstants, POZYX_TIMEOUT, DeviceRange,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, Quaternion)


class Positioning(Node):
    def __init__(self, anchors: List[DeviceCoordinates]):
        super().__init__("positioning")

        self.anchors = anchors

        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            self.get_logger().error("No Pozyx connected")
            return
        self.pozyx = PozyxSerial(serial_port)

        if self.handle_status(self.pozyx.clearDevices(), "Clearing Devices"):
            return

        if self.handle_status(self.pozyx.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES), "Discovering devices"):
            return

        print("Found devices:")
        self.pozyx.printDeviceList(include_coordinates=True)
        return

        for anchor in self.anchors:
            if anchor.flag != 1:
                self.get_logger().error("Anchor flags should be 1")
                return
            if self.handle_status(self.pozyx.addDevice(anchor), "Adding anchor"):
                return
        if len(self.anchors) > 4:
            if self.handle_status(
                self.pozyx.setSelectionOfAnchors(
                    PozyxConstants.ANCHOR_SELECT_AUTO,
                    len(self.anchors)
                ),
                "Handling more than 4 anchors"
            ):
                return
        if self.handle_status(self.pozyx.setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_PRECISION), "Setting Ranging Protocol"):
            return

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/pozyx/pose',
            10
        )

        self.log_config()
        self.thr = Thread(target=self.run)
        self.thr.start()

    def handle_status(self, status: int, info: str) -> bool:
        if status == POZYX_SUCCESS:
            return False

        if status == POZYX_TIMEOUT:
            timeout_msg = "; Timed Out"
        else:
            timeout_msg = ""

        err = SingleRegister()
        if self.pozyx.getErrorCode(err) != POZYX_SUCCESS:
            self.get_logger().error("Error getting error message")
            return True
        self.get_logger().error(
            f"{self.pozyx.getErrorMessage(err.value)}; {info}{timeout_msg}"
        )

        return True

    def log_config(self):
        list_size = SingleRegister()

        if self.handle_status(self.pozyx.getDeviceListSize(list_size), "Getting device list size"):
            return
        if list_size[0] != len(self.anchors):
            self.get_logger().error(
                f"Expected {len(self.anchors)} anchors, got {list_size.value}"
            )
            return
        device_list = DeviceList(list_size=list_size.value)
        if self.handle_status(self.pozyx.getDeviceIds(device_list), "Getting device IDs"):
            return

        for i in range(list_size.value):
            anchor_coordinates = Coordinates()
            if self.handle_status(self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates), "Getting device coordinates"):
                return
            self.get_logger().info(f"ANCHOR, {hex(device_list[i])}, {anchor_coordinates}")

    def run(self):
        try:
            while True:
                sleep(0.4)
                # if self.do_posing():
                #     return
                self.do_ranging()
        except KeyboardInterrupt:
            return

    def do_ranging(self):
        device_range = DeviceRange()
        for anchor in self.anchors:
            if self.handle_status(self.pozyx.doRanging(anchor.network_id, device_range), "Ranging"):
                continue
            self.get_logger().info(f"ANCHOR, {hex(anchor.network_id)}, {device_range}")

    def do_posing(self) -> bool:
        position = Coordinates()
        orientation = Quaternion()
        self.handle_status(self.pozyx.doPositioning(position, PozyxConstants.DIMENSION_2D, algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY), "Positioning")
        self.handle_status(self.pozyx.getNormalizedQuaternion(orientation), "Getting orientation")
        try:
            self.pose_pub.publish(
                PoseWithCovarianceStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id="pozyx_frame"
                    ),
                    pose=PoseWithCovariance(pose=Pose(
                        position=Point(x=position.x, y=position.y, z=position.z),
                        orientation=MsgQuaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
                    ))
                )
            )
        except Exception as e:
            if not e.args == ('cannot use Destroyable because destruction was requested',):
                raise e
            return True
        return False


def main(args=None):
    rclpy.init(args=args)

    node = Positioning([
        DeviceCoordinates(0x671a, 1, Coordinates(0, 600, 0)),
        DeviceCoordinates(0x762a, 1, Coordinates(900, 600, 0)),
        DeviceCoordinates(0x672d, 1, Coordinates(0, 0, 0)),
        DeviceCoordinates(0x6733, 1, Coordinates(780, 0, 0))
    ])

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    # rclpy.shutdown()
