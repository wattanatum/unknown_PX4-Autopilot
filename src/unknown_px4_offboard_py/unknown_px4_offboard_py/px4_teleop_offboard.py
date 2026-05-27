#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import Twist

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class PX4TeleopOffboard(Node):
    def __init__(self):
        super().__init__("px4_teleop_offboard")

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

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.vehicle_status_callback,
            self.px4_qos,
        )

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.vehicle_local_position_callback,
            self.px4_qos,
        )

        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack_v1",
            self.vehicle_command_ack_callback,
            self.px4_qos,
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.vehicle_status = None
        self.vehicle_local_position = None

        self.counter = 0
        self.phase = "TAKEOFF"

        # PX4 NED frame:
        # x forward, y right, z down.
        # z negative = altitude up.
        self.takeoff_z = -2.0

        # Position target in PX4 NED frame.
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = self.takeoff_z
        self.target_yaw = 0.0

        self.target_initialized = False

        self.last_cmd_time = 0.0
        self.last_timer_time = time.time()

        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_linear_z = 0.0
        self.cmd_yaw_rate = 0.0

        # Teleop speed limits.
        self.max_xy_speed = 0.7       # m/s
        self.max_z_speed = 0.5        # m/s
        self.max_yaw_rate = 0.8       # rad/s

        # If no key command arrives, hold the last target position.
        self.cmd_timeout_sec = 0.6

        # Position limits for safety.
        self.min_z = -5.0             # max altitude 5 m
        self.max_z = -0.5             # minimum altitude 0.5 m
        self.max_xy_distance = 10.0

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("PX4 position teleop offboard node started")
        self.get_logger().info("Phase 1: auto takeoff to 2 m")
        self.get_logger().info("Phase 2: position teleop")
        self.get_logger().info("t=up, b=down, i=forward, comma=back, j/l=yaw")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

        if not self.target_initialized:
            self.target_x = float(msg.x)
            self.target_y = float(msg.y)
            self.target_z = self.takeoff_z
            self.target_yaw = 0.0
            self.target_initialized = True

    def vehicle_command_ack_callback(self, msg):
        self.get_logger().info(
            f"Command ACK | command={msg.command}, result={msg.result}",
            throttle_duration_sec=1.0,
        )

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        self.cmd_linear_x = self.clamp(
            msg.linear.x,
            -self.max_xy_speed,
            self.max_xy_speed,
        )

        self.cmd_linear_y = self.clamp(
            msg.linear.y,
            -self.max_xy_speed,
            self.max_xy_speed,
        )

        self.cmd_linear_z = self.clamp(
            msg.linear.z,
            -self.max_z_speed,
            self.max_z_speed,
        )

        self.cmd_yaw_rate = self.clamp(
            msg.angular.z,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        self.get_logger().info(
            f"cmd_vel | x={self.cmd_linear_x:.2f}, y={self.cmd_linear_y:.2f}, "
            f"z={self.cmd_linear_z:.2f}, yaw={self.cmd_yaw_rate:.2f}",
            throttle_duration_sec=0.5,
        )

    def publish_offboard_control_mode_position(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_now()

        # Use position control for stable hold.
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

    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_now()

        msg.position = [
            float(self.target_x),
            float(self.target_y),
            float(self.target_z),
        ]

        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]

        if hasattr(msg, "jerk"):
            msg.jerk = [math.nan, math.nan, math.nan]

        msg.yaw = float(self.target_yaw)
        msg.yawspeed = math.nan

        self.trajectory_setpoint_pub.publish(msg)

    def update_teleop_position_target(self, dt):
        if time.time() - self.last_cmd_time > self.cmd_timeout_sec:
            ros_x = 0.0
            ros_y = 0.0
            ros_z = 0.0
            ros_yaw_rate = 0.0
        else:
            ros_x = self.cmd_linear_x
            ros_y = self.cmd_linear_y
            ros_z = self.cmd_linear_z
            ros_yaw_rate = self.cmd_yaw_rate

        # ROS teleop to PX4 NED:
        # ROS x forward -> NED x forward
        # ROS y left    -> NED y right is negative
        # ROS z up      -> NED z down is negative
        vx_ned = ros_x
        vy_ned = -ros_y
        vz_ned = -ros_z

        self.target_x += vx_ned * dt
        self.target_y += vy_ned * dt
        self.target_z += vz_ned * dt

        # If yaw direction feels reversed, remove the minus sign.
        self.target_yaw += -ros_yaw_rate * dt

        self.target_x = self.clamp(
            self.target_x,
            -self.max_xy_distance,
            self.max_xy_distance,
        )

        self.target_y = self.clamp(
            self.target_y,
            -self.max_xy_distance,
            self.max_xy_distance,
        )

        self.target_z = self.clamp(
            self.target_z,
            self.min_z,
            self.max_z,
        )

        self.target_yaw = self.wrap_pi(self.target_yaw)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("Arm command sent", throttle_duration_sec=1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info("Offboard mode command sent", throttle_duration_sec=1.0)

    def land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )
        self.get_logger().info("Land command sent")

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
        msg.timestamp = self.timestamp_now()

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def is_armed(self):
        if self.vehicle_status is None:
            return False

        return self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_offboard(self):
        if self.vehicle_status is None:
            return False

        return self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def reached_takeoff_height(self):
        if self.vehicle_local_position is None:
            return False

        # PX4 NED: z negative = up.
        # Switch to teleop after reaching about 1.8 m.
        return self.vehicle_local_position.z < -1.8

    def timer_callback(self):
        now = time.time()
        dt = now - self.last_timer_time
        self.last_timer_time = now

        dt = self.clamp(dt, 0.0, 0.1)
        self.counter += 1

        self.publish_offboard_control_mode_position()

        if self.phase == "TAKEOFF":
            self.target_z = self.takeoff_z

            if self.counter > 40:
                if not self.is_offboard():
                    self.engage_offboard_mode()

                if not self.is_armed():
                    self.arm()

            if self.reached_takeoff_height():
                self.phase = "TELEOP"
                self.get_logger().info("Takeoff complete. Switching to TELEOP position mode.")

        elif self.phase == "TELEOP":
            self.update_teleop_position_target(dt)

        self.publish_position_setpoint()

        if self.vehicle_status is not None:
            self.get_logger().info(
                f"phase={self.phase} | nav_state={self.vehicle_status.nav_state}, "
                f"arming_state={self.vehicle_status.arming_state}",
                throttle_duration_sec=2.0,
            )

        if self.vehicle_local_position is not None:
            self.get_logger().info(
                f"Local position | x={self.vehicle_local_position.x:.2f}, "
                f"y={self.vehicle_local_position.y:.2f}, "
                f"z={self.vehicle_local_position.z:.2f} | "
                f"Target | x={self.target_x:.2f}, y={self.target_y:.2f}, z={self.target_z:.2f}",
                throttle_duration_sec=2.0,
            )

    def timestamp_now(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    @staticmethod
    def wrap_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


def main(args=None):
    rclpy.init(args=args)

    node = PX4TeleopOffboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping PX4 teleop offboard node")
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
