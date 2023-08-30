import rclpy
from rclpy.node import Node
# from rclpy.action import ActionServer
# from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import ParameterDescriptor

from pyvesc import VESC
from serial import SerialException
from threading import Event, Thread
from mining_arm.angle_sense import AngleSensor
from multiprocessing import Value

# from global_msgs.action import SetArmAngle
from std_msgs.msg import Float32
from py_globals.wrappers import ErrorWrapper


class MiningArm(Node):
    def __init__(self):
        super().__init__("mining_arm")

        self.declare_parameter(
            "arm_motor_port",
            "/dev/fsesc_arm",
            ParameterDescriptor(
                description="The serial port that the arm motor"
                " is connected to"
            )
        )
        self.declare_parameter(
            "drum_motor_port",
            "/dev/fsesc_drum",
            ParameterDescriptor(
                description="The serial port that the drum motor"
                " is connected to"
            )
        )
        self.declare_parameter(
            "arm_speed_scale",
            0.6,
            ParameterDescriptor(
                description="A scale factor for the speed of the arm"
            )
        )
        self.declare_parameter(
            "angle_read_rate",
            50,
            ParameterDescriptor(
                description="The rate at which the angle sensor reads at in Hz"
            )
        )
        self.declare_parameter(
            "max_arm_angle",
            18.0,
            ParameterDescriptor(
                description="The maximum angle that the arm can safely move to"
            )
        )
        self.declare_parameter(
            "min_arm_angle",
            -37.0,
            ParameterDescriptor(
                description="The maximum angle that the arm can safely move to"
            )
        )
        self.declare_parameter(
            "angle_offset",
            -147.0,
            ParameterDescriptor(
                description="The angle to offset the current angle by"
            )
        )

        def on_err(e: Exception):
            self.get_logger().debug(str(e))

        self.arm_motor = ErrorWrapper(
            lambda: VESC(
                serial_port=self.get_parameter("arm_motor_port")
                .get_parameter_value()
                .string_value
            ),
            SerialException,
            on_err
        )

        self.drum_motor = ErrorWrapper(
            lambda: VESC(
                serial_port=self.get_parameter("drum_motor_port")
                .get_parameter_value()
                .string_value
            ),
            SerialException,
            on_err
        )

        self.arm_angle = 0
        self.arm_angle_update_event = Event()
        self.updating_arm_angle = True

        def update_angle():
            angle_sensor = AngleSensor()
            rate = self.create_rate(
                self.get_parameter("angle_read_rate")
                .get_parameter_value()
                .integer_value
            )
            offset = self.get_parameter("angle_offset") \
                .get_parameter_value()  \
                .double_value
            arm_angle_pub = self.create_publisher(
                Float32,
                "arm_angle",
                10
            )

            while self.updating_arm_angle:
                self.arm_angle = angle_sensor.get_angle() + offset
                self.arm_angle_update_event.set()
                self.arm_angle_update_event.clear()
                arm_angle_pub.publish(Float32(data=self.arm_angle))
                rate.sleep()

        Thread(
            target=update_angle
        ).start()

        # self.setting_arm_angle = False
        # self.cancel_set_arm_angle = False
        # self.set_cancellation = Event()

        # self.set_arm_angle_server = ActionServer(
        #     self,
        #     SetArmAngle,
        #     "set_arm_angle",
        #     self.set_arm_angle_callback,
        # )

        # self.is_arm_vel_set = False
        self.set_arm_velocity_sub = self.create_subscription(
            Float32,
            "target_arm_velocity",
            self.set_arm_velocity,
            10
        )

        self.set_drum_velocity_sub = self.create_subscription(
            Float32,
            "target_drum_velocity",
            self.set_drum_velocity,
            10
        )

        from ctypes import c_float
        self.arm_velocity = Value(c_float, 0.0)

        def send_arm_velocity(arm_vel):
            max_angle = self.get_parameter("max_arm_angle") \
                .get_parameter_value()  \
                .double_value
            min_angle = self.get_parameter("min_arm_angle") \
                .get_parameter_value()  \
                .double_value
            scale = self.get_parameter("arm_speed_scale") \
                .get_parameter_value()  \
                .double_value

            while True:
                vel = arm_vel.value
                if vel > 0:
                    if self.arm_angle >= max_angle:
                        self.arm_motor.exec(lambda x: x.set_duty_cycle(0))
                        continue
                elif self.arm_angle < min_angle:
                    self.arm_motor.exec(lambda x: x.set_duty_cycle(0))
                    continue

                self.arm_motor.exec(lambda x: x.set_duty_cycle(vel * scale))

        Thread(
            target=send_arm_velocity,
            args=(
                self.arm_velocity,
            )
        ).start()

    def close(self):
        # self.updating_arm_angle = False
        self.arm_motor.exec(lambda x: x.stop_heartbeat())
        self.drum_motor.exec(lambda x: x.stop_heartbeat())

    # def set_arm_angle_callback(self, goal_handle: ServerGoalHandle):
    #     if self.setting_arm_angle:
    #         self.cancel_set_arm_angle = True
    #         self.set_cancellation.clear()
    #         self.set_cancellation.wait()

    #     self.setting_arm_angle = True
    #     self.cancel_set_arm_angle = False
    #     self.is_arm_vel_set = False

    #     less_than = self.arm_angle < goal_handle.request.target_angle

    #     # TODO check values
    #     self.arm_motor.set_duty_cycle(1 if less_than else -1)

    #     while True:
    #         self.arm_angle_update_event.wait()

    #         diff = goal_handle.request.target_angle - self.arm_angle
    #         feedback = SetArmAngle.Feedback()
    #         feedback.difference = diff
    #         goal_handle.publish_feedback(feedback)

    #         if goal_handle.is_cancel_requested() or \
    #                 self.is_arm_vel_set or \
    #                 self.cancel_set_arm_angle:

    #             self.set_cancellation.set()
    #             self.setting_arm_angle = False

    #             if goal_handle.is_cancel_requested():
    #                 self.arm_motor.set_duty_cycle(0)

    #             goal_handle.canceled()
    #             return SetArmAngle.Result()

    #         if less_than:
    #             if diff >= 0:
    #                 break
    #         elif diff <= 0:
    #             break

    #     goal_handle.succeed()
    #     self.arm_motor.set_duty_cycle(0)
    #     self.setting_arm_angle = False

    #     return SetArmAngle.Result()

    def set_arm_velocity(self, msg):
        if not -1 <= msg.data <= 1:
            self.get_logger().error(
                f"Received out of bounds arm velocity: {msg.data}"
            )
            return

        self.arm_velocity.value = msg.data

    def set_drum_velocity(self, msg):
        if not -1 <= msg.data <= 1:
            self.get_logger().error(
                f"Received out of bounds drum velocity: {msg.data}"
            )
            return
        self.drum_motor.exec(lambda x: x.set_duty_cycle(msg.data))


def main():
    rclpy.init()
    node = MiningArm()
    rclpy.spin(node)
    node.close()


if __name__ == "__main__":
    main()
