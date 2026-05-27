#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__("offboard_takeoff")

        # PX4 uXRCE-DDS topics usually use BEST_EFFORT.
        # If we use default RELIABLE subscriber QoS, ROS 2 will show:
        # "offering incompatible QoS ... RELIABILITY"
        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            self.px4_qos,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self.px4_qos,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            self.px4_qos,
        )

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.vehicle_local_position_callback,
            self.px4_qos,
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.vehicle_status_callback,
            self.px4_qos,
        )

        self.vehicle_local_position = None
        self.vehicle_status = None

        self.offboard_setpoint_counter = 0

        # PX4 uses NED frame:
        # x = forward
        # y = right
        # z = down
        #
        # z = -2.0 means 2 meters above takeoff point.
        self.takeoff_height = -2.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("PX4 offboard takeoff node started")

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        if hasattr(msg, "thrust_and_torque"):
            msg.thrust_and_torque = False

        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Hold x=0, y=0, z=-2m.
        msg.position = [0.0, 0.0, self.takeoff_height]

        msg.velocity = [float("nan"), float("nan"), float("nan")]
        msg.acceleration = [float("nan"), float("nan"), float("nan")]

        msg.yaw = 0.0
        msg.yawspeed = float("nan")

        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Send setpoints first before Offboard mode.
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # Print status once data arrives.
        if self.vehicle_status is not None and self.offboard_setpoint_counter == 11:
            nav_state = self.vehicle_status.nav_state
            arming_state = self.vehicle_status.arming_state
            self.get_logger().info(
                f"PX4 status received | nav_state={nav_state}, arming_state={arming_state}",
                throttle_duration_sec=2.0,
            )

        if self.vehicle_local_position is not None and self.offboard_setpoint_counter == 11:
            self.get_logger().info(
                f"Local position | x={self.vehicle_local_position.x:.2f}, "
                f"y={self.vehicle_local_position.y:.2f}, "
                f"z={self.vehicle_local_position.z:.2f}",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)

    node = OffboardTakeoff()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping offboard node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
