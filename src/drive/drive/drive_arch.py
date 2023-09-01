import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from serial import Serial, SerialException
# from threading import Thread
from pyvesc import VESC

from drive.drive_calculator import drive_steering

from global_msgs.msg import Steering
from py_globals.wrappers import ErrorWrapper


class Drive(Node):
    DUTY_FREQUENCY = 10000
    WAIT_FOR_READY_DURATION = 3

    def __init__(self):
        super().__init__("drive")

        def on_err(e: Exception):
            self.get_logger().error(str(e))

        self.spare_controller = ErrorWrapper(
            lambda: VESC(serial_port="/dev/spare_motor"),
            SerialException,
            on_err
        )

        self.declare_parameter(
            "controller_port",
            "/dev/drive_control",
            ParameterDescriptor(
                description="The port of the motor controller"
            )
        )

        def init_controller():
            controller = Serial(
                self.get_parameter("controller_port")
                    .get_parameter_value()
                    .string_value,
                115200
            )
            controller.write("printReadies()\r".encode())
            controller.readline()
            unready = []

            for _ in range(6):
                line = controller.readline().decode()
                if "0" in line:
                    unready.append(line)

            if len(unready) > 3:
                for line in unready:
                    self.get_logger().error(line)
                return
            return controller

        self.controller = ErrorWrapper(
            init_controller,
            SerialException,
            on_err
        )

        self.movement_listener = self.create_subscription(
            Steering,
            'steering',
            self.movement_callback,
            50
        )

        # def logger():
        #     while True:
        #         self.get_logger().info(self.controller.readline().decode())

        # Thread(target=logger).start()

    def movement_callback(self, msg):
        left_drive, right_drive = drive_steering(msg.drive, msg.steering)
        self.set_drive(left_drive, right_drive)

    def set_drive(self, left_drive: float, right_drive: float):
        if not -1 <= left_drive <= 1:
            self.get_logger().error(
                f"Received invalid left drive of: {left_drive}"
            )
            return
        if not -1 <= right_drive <= 1:
            self.get_logger().error(
                f"Received invalid right drive of: {right_drive}"
            )
            return

        # self.get_logger().info(f'setSpeed({left_drive},{right_drive})')

        if abs(right_drive) < 0.01 and abs(left_drive) < 0.01:
            def run(x: Serial):
                x.write(b'setEnable(0)\r')
                x.readline()
            self.controller.exec(run)

            self.spare_controller.exec(lambda x: x.set_duty_cycle(0.0))

        else:
            def run(x: Serial):
                x.write((
                    f"s(1,{int(left_drive > 0)},{int(right_drive > 0)},"
                    f"{left_drive},{right_drive})\r"
                ).encode())
                x.readline()
            self.controller.exec(run)

            self.spare_controller.exec(lambda x: x.set_duty_cycle(right_drive))


def main():
    rclpy.init()
    drive = Drive()
    rclpy.spin(drive)
    drive.controller.close()


if __name__ == "__main__":
    main()
