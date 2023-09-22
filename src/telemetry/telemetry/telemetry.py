import rclpy
import enet
from enum import IntEnum
from rclpy.node import Node
from struct import Struct
from global_msgs.msg import Steering
from global_msgs.msg import CompressedImagePacket
from std_msgs.msg import Float32
from multiprocessing import Process, Queue, Value


class Channels(IntEnum):
    IMPORTANT = 0
    CAMERA = 1
    ODOMETRY = 2
    STEERING = 3
    MAX = 4


class ImportantMessage(IntEnum):
    ENABLE_CAMERA = 0
    DISABLE_CAMERA = 1


class ControlScheme(IntEnum):
    ARCHIMEDES = 0
    UNNAMED = 1


class Telemetry(Node):
    def __init__(self, scheme: int):
        super().__init__("telemetry")
        self.address = b"127.0.0.1"
        self.port = 43721
        self.steering_pub = self.create_publisher(Steering, 'steering', 10)
        self.arm_vel_pub = self.create_publisher(
            Float32, 'target_arm_velocity', 10
        )
        self.camera_image_buffer = Queue()
        self.camera_listener = self.create_subscription(
            CompressedImagePacket,
            'compressed_image',
            self.receive_image,
            10
        )

        self.scheme = scheme
        if scheme == ControlScheme.ARCHIMEDES:
            self.steering_struct = Struct("bbbb")
            self.drum_vel_pub = self.create_publisher(
                Float32, 'target_drum_velocity', 10
            )
        else:
            self.steering_struct = Struct("bbb")

        self.connected = Value('b', False)
        self.thr = Process(target=self.run)
        self.thr.start()

    def receive_image(self, img: CompressedImagePacket):
        with self.connected.get_lock():
            if not self.connected.value:
                return

        self.camera_image_buffer.put(bytes(img.data))

    def run(self):
        logger = self.get_logger()
        host = enet.Host(None, 1, 0, 0, 0)
        addr = enet.Address(self.address, self.port)

        logger.info("Connecting to lunabase...")
        while True:
            peer = host.connect(addr, Channels.MAX)

            # Connect Loop
            while True:
                event = host.service(1000)
                if event.type == enet.EVENT_TYPE_CONNECT:
                    if event.peer.address != peer.address:
                        logger.warn("Somehow connected to wrong peer")
                        event.peer.disconnect()
                        continue
                    break

                elif event.type == enet.EVENT_TYPE_DISCONNECT:
                    peer = host.connect(addr, Channels.MAX)

                elif event.type != enet.EVENT_TYPE_NONE:
                    logger.warn(f"Received non-connect event: {event.type}")

            logger.info("Connected to lunabase!")

            # Empty camera buffer
            while not self.camera_image_buffer.empty():
                self.camera_image_buffer.get()

            with self.connected.get_lock():
                self.connected.value = True

            # Main Loop
            while True:
                event = host.service(1000)

                if event.type == enet.EVENT_TYPE_CONNECT:
                    logger.warn("Somehow connected to a peer:"
                                f"{event.peer.address}")

                elif event.type == enet.EVENT_TYPE_DISCONNECT:
                    if event.peer.address != peer.address:
                        logger.warn(
                            "Somehow disconnected from a peer:"
                            f"{event.peer.address}"
                        )
                        continue
                    with self.connected.get_lock():
                        self.connected.value = True
                    logger.error("Disconnected from lunabase!")
                    break

                elif event.type == enet.EVENT_TYPE_RECEIVE:
                    if event.peer.address != peer.address:
                        logger.warn(
                            "Somehow received from a different peer:"
                            f"{event.peer.address}"
                        )
                        continue
                    self.on_receive(event.channelID, event.packet.data, peer)

                elif event.type == -1:
                    host.destroy()
                    host = enet.Host(None, 1, 0, 0, 0)
                    logger.error("Host has returned an error! Restarting...")
                    break

                data = bytearray()
                while not self.camera_image_buffer.empty():
                    data += self.camera_image_buffer.get()

                    while len(data) >= 512:
                        peer.send(
                            Channels.CAMERA,
                            enet.Packet(
                                data[0:512],
                                enet.PACKET_FLAG_UNSEQUENCED | enet.PACKET_FLAG_UNRELIABLE_FRAGMENT
                            )
                        )
                        del data[0:512]

                if len(data) > 0:
                    peer.send(
                        Channels.CAMERA,
                        enet.Packet(
                            data,
                            enet.PACKET_FLAG_UNSEQUENCED | enet.PACKET_FLAG_UNRELIABLE_FRAGMENT
                        )
                    )

    def on_receive(self, channel: int, data: bytes, peer) -> None:
        logger = self.get_logger()

        if channel == Channels.IMPORTANT:
            pass

        elif channel == Channels.STEERING:
            peer.send(
                Channels.STEERING,
                enet.Packet(data, enet.PACKET_FLAG_UNSEQUENCED)
            )
            result = self.steering_struct.unpack(data)

            if self.scheme == ControlScheme.ARCHIMEDES:
                drive, steering, arm_vel, drum_vel = result

                msg = Float32()
                msg.data = drum_vel / 127
                self.drum_vel_pub.publish(msg)

            else:
                drive, steering, arm_vel = result

            msg = Steering()
            msg.drive = drive / 127
            msg.steering = steering / 127
            self.steering_pub.publish(msg)

            msg = Float32()
            msg.data = arm_vel / 127
            self.arm_vel_pub.publish(msg)

        else:
            logger.warn(f"Unexpected channel: {channel}")


def main(args=None):
    rclpy.init(args=args)

    node = Telemetry(ControlScheme.UNNAMED)

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


def main_arch(args=None):
    rclpy.init(args=args)

    node = Telemetry(ControlScheme.ARCHIMEDES)

    rclpy.spin(node)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    raise Exception("Please run from ROS 2")
