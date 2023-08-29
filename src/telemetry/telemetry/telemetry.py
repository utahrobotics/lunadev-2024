import rclpy
from threading import Thread
import enet
from enum import IntEnum
from rclpy.node import Node
from struct import Struct
from global_msgs.msg import Steering


class Channels(IntEnum):
    IMPORTANT = 0
    CAMERA = 1
    ODOMETRY = 2
    STEERING = 3
    MAX = 4


class ImportantMessage(IntEnum):
    ENABLE_CAMERA = 0
    DISABLE_CAMERA = 1


class Telemetry(Node):
    def __init__(self):
        super().__init__("telemetry")
        self.address = b"host.docker.internal"
        self.port = 43721
        self.thr = Thread(target=self.run)
        self.thr.start()
        self.steering_struct = Struct("bb")
        self.steering_pub = self.create_publisher(Steering, 'steering', 10)

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

    def on_receive(self, channel: int, data: bytes, peer) -> None:
        logger = self.get_logger()

        if channel == Channels.IMPORTANT:
            pass

        elif channel == Channels.STEERING:
            peer.send(
                Channels.STEERING,
                enet.Packet(data, enet.PACKET_FLAG_UNSEQUENCED)
            )
            drive, steering = self.steering_struct.unpack(data)
            msg = Steering()
            msg.drive = drive / 127
            msg.steering = steering / 127
            self.steering_pub.publish(msg)

        else:
            logger.warn(f"Unexpected channel: {channel}")


def main(args=None):
    rclpy.init(args=args)

    node = Telemetry()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
