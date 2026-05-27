#!/usr/bin/env python3

import asyncio
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


class CmdVelToPX4(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_px4")

        # ROS topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("odom_topic", "/odom")

        # PX4 connection
        self.declare_parameter("system_address", "udpin://0.0.0.0:14540")
        self.declare_parameter("auto_arm", True)

        # Altitude control
        self.declare_parameter("target_altitude", 1.5)
        self.declare_parameter("altitude_kp", 0.18)
        self.declare_parameter("max_vertical_speed", 0.10)

        # Forward / backward / yaw limits
        self.declare_parameter("max_forward_speed", 0.25)
        self.declare_parameter("max_backward_speed", 0.08)
        self.declare_parameter("min_forward_speed", 0.00)

        self.declare_parameter("max_yaw_speed_deg", 18.0)
        self.declare_parameter("min_yaw_speed_deg", 0.0)

        # Ignore tiny yaw corrections from Nav2
        self.declare_parameter("yaw_deadband_deg", 1.0)

        self.declare_parameter("invert_yaw", True)
        self.declare_parameter("smoothing_alpha", 0.18)

        # Timeout / reset behavior
        self.declare_parameter("cmd_timeout", 1.0)

        # Usually keep false because immediate reset on every new path can shake drone
        self.declare_parameter("reset_on_new_path", False)
        self.declare_parameter("path_reset_duration", 0.35)
        self.declare_parameter("path_reset_cooldown", 3.0)

        # Better reset behavior: reset only if drone is stuck after new path
        self.declare_parameter("reset_if_stuck_after_new_path", True)
        self.declare_parameter("stuck_check_duration", 3.0)
        self.declare_parameter("stuck_movement_threshold", 0.03)
        self.declare_parameter("stuck_cmd_threshold", 0.005)
        self.declare_parameter("stuck_reset_duration", 0.50)
        self.declare_parameter("stuck_reset_cooldown", 5.0)

        # Read parameters
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.system_address = self.get_parameter("system_address").value
        self.auto_arm = bool(self.get_parameter("auto_arm").value)

        self.target_altitude = float(self.get_parameter("target_altitude").value)
        self.altitude_kp = float(self.get_parameter("altitude_kp").value)
        self.max_vertical_speed = float(self.get_parameter("max_vertical_speed").value)

        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_backward_speed = float(self.get_parameter("max_backward_speed").value)
        self.min_forward_speed = float(self.get_parameter("min_forward_speed").value)

        self.max_yaw_speed_deg = float(self.get_parameter("max_yaw_speed_deg").value)
        self.min_yaw_speed_deg = float(self.get_parameter("min_yaw_speed_deg").value)
        self.yaw_deadband_deg = float(self.get_parameter("yaw_deadband_deg").value)

        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.reset_on_new_path = bool(self.get_parameter("reset_on_new_path").value)
        self.path_reset_duration = float(self.get_parameter("path_reset_duration").value)
        self.path_reset_cooldown = float(self.get_parameter("path_reset_cooldown").value)

        self.reset_if_stuck_after_new_path = bool(
            self.get_parameter("reset_if_stuck_after_new_path").value
        )
        self.stuck_check_duration = float(self.get_parameter("stuck_check_duration").value)
        self.stuck_movement_threshold = float(
            self.get_parameter("stuck_movement_threshold").value
        )
        self.stuck_cmd_threshold = float(self.get_parameter("stuck_cmd_threshold").value)
        self.stuck_reset_duration = float(self.get_parameter("stuck_reset_duration").value)
        self.stuck_reset_cooldown = float(self.get_parameter("stuck_reset_cooldown").value)

        # MAVSDK drone
        self.drone = System()

        # Telemetry state
        self.current_relative_altitude = None
        self.current_flight_mode = None

        # Odom state
        self.current_odom_x = None
        self.current_odom_y = None

        # Runtime state
        self.offboard_started = False
        self.running = True

        # Target commands from Nav2
        self.target_forward = 0.0
        self.target_yaw_deg = 0.0

        # Smoothed commands sent to PX4
        self.forward = 0.0
        self.right = 0.0
        self.yaw_deg = 0.0

        # Timeout / reset state
        self.last_cmd_time = time.time()

        self.last_path_signature = None
        self.last_path_reset_time = 0.0
        self.reset_until_time = 0.0

        # Stuck-watch state after new path
        self.watch_new_path = False
        self.new_path_time = 0.0
        self.new_path_start_x = None
        self.new_path_start_y = None
        self.last_stuck_reset_time = 0.0
        self.stuck_reset_until_time = 0.0

        # ROS subscriptions
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(f"Subscribing to cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"Subscribing to Nav2 path: {self.path_topic}")
        self.get_logger().info(f"Subscribing to odom: {self.odom_topic}")
        self.get_logger().info("PX4 Offboard controller for Nav2 /cmd_vel")
        self.get_logger().info(f"Target altitude: {self.target_altitude:.2f} m")
        self.get_logger().info(f"Max forward speed: {self.max_forward_speed:.2f} m/s")
        self.get_logger().info(f"Max backward speed: {self.max_backward_speed:.2f} m/s")
        self.get_logger().info(f"Max yaw speed: {self.max_yaw_speed_deg:.1f} deg/s")
        self.get_logger().info(f"Yaw deadband: {self.yaw_deadband_deg:.1f} deg/s")
        self.get_logger().info(f"Invert yaw: {self.invert_yaw}")
        self.get_logger().info(f"Reset on new path immediately: {self.reset_on_new_path}")
        self.get_logger().info(
            f"Reset if stuck after new path: {self.reset_if_stuck_after_new_path}"
        )

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def reset_control_state(self, reason="manual"):
        self.target_forward = 0.0
        self.target_yaw_deg = 0.0

        self.forward = 0.0
        self.right = 0.0
        self.yaw_deg = 0.0

        self.get_logger().warn(f"PX4 Offboard control state reset: {reason}")

    def make_path_signature(self, msg: Path):
        if len(msg.poses) == 0:
            return None

        first = msg.poses[0].pose.position
        mid = msg.poses[len(msg.poses) // 2].pose.position
        last = msg.poses[-1].pose.position

        return (
            len(msg.poses),
            round(first.x, 1),
            round(first.y, 1),
            round(mid.x, 1),
            round(mid.y, 1),
            round(last.x, 1),
            round(last.y, 1),
        )

    def path_callback(self, msg: Path):
        signature = self.make_path_signature(msg)
        if signature is None:
            return

        now = time.time()

        if self.last_path_signature is None:
            self.last_path_signature = signature
            self.get_logger().info("First Nav2 path received")
            return

        if signature == self.last_path_signature:
            return

        self.last_path_signature = signature
        self.get_logger().warn("New Nav2 path detected")

        # Option A: immediate reset on new path. Usually disabled.
        if self.reset_on_new_path:
            if now - self.last_path_reset_time >= self.path_reset_cooldown:
                self.last_path_reset_time = now
                self.reset_until_time = now + self.path_reset_duration
                self.reset_control_state("new Nav2 path received")
            else:
                self.get_logger().info(
                    "Immediate path reset skipped because cooldown is active",
                    throttle_duration_sec=1.0,
                )

        # Option B: start stuck watch after new path.
        if self.reset_if_stuck_after_new_path:
            self.watch_new_path = True
            self.new_path_time = now
            self.new_path_start_x = self.current_odom_x
            self.new_path_start_y = self.current_odom_y

            self.get_logger().warn("Starting stuck watch after new path")

    def odom_callback(self, msg: Odometry):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        # Nav2:
        # linear.x  = forward/backward velocity in m/s
        # angular.z = yaw velocity in rad/s

        # Forward is allowed faster than backward.
        if msg.linear.x >= 0.0:
            forward = self.clamp(
                msg.linear.x,
                0.0,
                self.max_forward_speed,
            )
        else:
            forward = self.clamp(
                msg.linear.x,
                -self.max_backward_speed,
                0.0,
            )

        # Optional boost for tiny positive forward command only.
        # Do not boost negative command because reverse should remain gentle.
        if 0.001 < forward < self.min_forward_speed:
            forward = self.min_forward_speed

        yaw_deg_raw = math.degrees(msg.angular.z)

        # Ignore tiny yaw corrections from Nav2.
        if abs(yaw_deg_raw) < self.yaw_deadband_deg:
            yaw_deg = 0.0
        else:
            yaw_deg = yaw_deg_raw

        yaw_deg = self.clamp(
            yaw_deg,
            -self.max_yaw_speed_deg,
            self.max_yaw_speed_deg,
        )

        # Optional boost for tiny non-zero yaw command.
        # Usually keep min_yaw_speed_deg = 0.0 to avoid shaking.
        if abs(yaw_deg) > 0.5 and abs(yaw_deg) < self.min_yaw_speed_deg:
            yaw_deg = self.min_yaw_speed_deg if yaw_deg > 0.0 else -self.min_yaw_speed_deg

        # ROS angular.z and PX4 body yaw may be opposite.
        if self.invert_yaw:
            yaw_deg = -yaw_deg

        self.target_forward = forward
        self.target_yaw_deg = yaw_deg

        self.get_logger().info(
            f"Nav2 cmd_vel: linear.x={msg.linear.x:.3f}, "
            f"angular.z={msg.angular.z:.3f}, yaw_raw={yaw_deg_raw:.1f} deg/s | "
            f"PX4 target: forward={self.target_forward:.2f}, "
            f"yaw={self.target_yaw_deg:.1f} deg/s",
            throttle_duration_sec=1.0,
        )

    def odom_distance_since_new_path(self):
        if self.current_odom_x is None or self.current_odom_y is None:
            return None

        if self.new_path_start_x is None or self.new_path_start_y is None:
            return None

        dx = self.current_odom_x - self.new_path_start_x
        dy = self.current_odom_y - self.new_path_start_y

        return math.sqrt(dx * dx + dy * dy)

    def should_reset_because_stuck(self):
        if not self.reset_if_stuck_after_new_path:
            return False

        if not self.watch_new_path:
            return False

        now = time.time()

        if now - self.new_path_time < self.stuck_check_duration:
            return False

        if now - self.last_stuck_reset_time < self.stuck_reset_cooldown:
            self.watch_new_path = False
            self.get_logger().info(
                "Stuck reset skipped because cooldown is active",
                throttle_duration_sec=1.0,
            )
            return False

        command_active = (
            abs(self.target_forward) > self.stuck_cmd_threshold
            or abs(self.target_yaw_deg) > self.yaw_deadband_deg
        )

        if not command_active:
            self.watch_new_path = False
            self.get_logger().info(
                "Stuck watch ended: no active command from Nav2",
                throttle_duration_sec=1.0,
            )
            return False

        distance = self.odom_distance_since_new_path()

        if distance is None:
            self.get_logger().warn(
                "Stuck watch cannot check movement because odom is not ready",
                throttle_duration_sec=1.0,
            )
            return False

        if distance < self.stuck_movement_threshold:
            self.last_stuck_reset_time = now
            self.stuck_reset_until_time = now + self.stuck_reset_duration
            self.watch_new_path = False

            self.reset_control_state(
                f"stuck after new path: moved only {distance:.3f} m"
            )

            return True

        self.watch_new_path = False
        self.get_logger().info(
            f"Stuck watch ended: drone moved {distance:.3f} m"
        )
        return False

    async def altitude_loop(self):
        async for position in self.drone.telemetry.position():
            self.current_relative_altitude = position.relative_altitude_m

            self.get_logger().info(
                f"Altitude: {self.current_relative_altitude:.2f} / "
                f"target {self.target_altitude:.2f}",
                throttle_duration_sec=2.0,
            )

            if not self.running:
                break

    async def flight_mode_loop(self):
        async for mode in self.drone.telemetry.flight_mode():
            self.current_flight_mode = str(mode)

            self.get_logger().info(
                f"PX4 flight mode: {self.current_flight_mode}",
                throttle_duration_sec=2.0,
            )

            if not self.running:
                break

    def altitude_down_velocity(self):
        if self.current_relative_altitude is None:
            return 0.0

        # NED:
        # down > 0 = descend
        # down < 0 = ascend
        error = self.target_altitude - self.current_relative_altitude
        down = -self.altitude_kp * error

        return self.clamp(
            down,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )

    def smooth_nav2_commands(self):
        alpha = self.smoothing_alpha

        self.forward = (1.0 - alpha) * self.forward + alpha * self.target_forward
        self.yaw_deg = (1.0 - alpha) * self.yaw_deg + alpha * self.target_yaw_deg

        if abs(self.target_forward) < 0.001 and abs(self.forward) < 0.02:
            self.forward = 0.0

        if (
            abs(self.target_yaw_deg) < self.yaw_deadband_deg
            and abs(self.yaw_deg) < self.yaw_deadband_deg
        ):
            self.yaw_deg = 0.0

    async def connect_px4(self):
        await self.drone.connect(system_address=self.system_address)

        self.get_logger().info("Waiting for PX4 connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("PX4 connected")
                break

        self.get_logger().info("Waiting for PX4 health...")
        async for health in self.drone.telemetry.health():
            self.get_logger().info(
                f"Health global={health.is_global_position_ok} "
                f"home={health.is_home_position_ok} "
                f"local={health.is_local_position_ok}",
                throttle_duration_sec=2.0,
            )

            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("PX4 health OK")
                break

            await asyncio.sleep(1.0)

    async def send_initial_offboard_setpoints(self):
        self.get_logger().info("Sending initial Offboard setpoints before mode switch")

        for _ in range(100):
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            )
            await asyncio.sleep(0.05)

    async def start_offboard(self):
        for attempt in range(1, 8):
            try:
                self.get_logger().info(f"Starting Offboard attempt {attempt}/7")
                await self.drone.offboard.start()
                self.offboard_started = True
                self.get_logger().info("Offboard command accepted")
                await asyncio.sleep(1.0)

                self.reset_control_state("Offboard started")
                return True

            except OffboardError as error:
                self.get_logger().error(
                    f"Offboard start failed: {error._result.result}"
                )
                self.offboard_started = False

                for _ in range(20):
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        )
                    )
                    await asyncio.sleep(0.05)

        return False

    async def arm_and_enter_offboard(self):
        await self.connect_px4()

        asyncio.create_task(self.altitude_loop())
        asyncio.create_task(self.flight_mode_loop())

        await self.send_initial_offboard_setpoints()

        if self.auto_arm:
            try:
                self.get_logger().info("Arming")
                await self.drone.action.arm()
            except ActionError as error:
                self.get_logger().error(f"Arm failed: {error._result.result}")
                self.get_logger().error("Check PX4 terminal: commander check")
                return False

        ok = await self.start_offboard()

        if not ok:
            self.get_logger().error("Could not enter Offboard")
            return False

        self.get_logger().info("Climbing in Offboard mode")

        while self.running:
            if self.current_relative_altitude is not None:
                if self.current_relative_altitude >= self.target_altitude - 0.15:
                    self.get_logger().info("Target altitude reached")
                    break

            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    -0.25,
                    0.0,
                )
            )
            await asyncio.sleep(0.05)

        self.reset_control_state("target altitude reached")
        return True

    async def send_velocity_loop(self):
        while self.running:
            if self.offboard_started:
                try:
                    now = time.time()

                    # Clear old command if Nav2 stops publishing.
                    if now - self.last_cmd_time > self.cmd_timeout:
                        self.target_forward = 0.0
                        self.target_yaw_deg = 0.0

                    down = self.altitude_down_velocity()

                    # Check stuck condition after new path.
                    self.should_reset_because_stuck()

                    # Hold neutral during immediate path reset or stuck reset.
                    if now < self.reset_until_time or now < self.stuck_reset_until_time:
                        await self.drone.offboard.set_velocity_body(
                            VelocityBodyYawspeed(
                                0.0,
                                0.0,
                                down,
                                0.0,
                            )
                        )

                        self.get_logger().warn(
                            "Holding neutral command for Offboard reset",
                            throttle_duration_sec=0.5,
                        )

                        await asyncio.sleep(0.05)
                        continue

                    self.smooth_nav2_commands()

                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(
                            self.forward,
                            self.right,
                            down,
                            self.yaw_deg,
                        )
                    )

                    self.get_logger().info(
                        f"PX4 cmd: forward={self.forward:.2f}, "
                        f"down={down:.2f}, "
                        f"yaw={self.yaw_deg:.1f}",
                        throttle_duration_sec=1.0,
                    )

                except OffboardError as error:
                    self.get_logger().error(
                        f"Offboard velocity failed: {error._result.result}"
                    )
                    self.offboard_started = False

                    self.reset_control_state("Offboard velocity failed")
                    await self.send_initial_offboard_setpoints()
                    await self.start_offboard()

            await asyncio.sleep(0.05)

    async def run_async(self):
        ok = await self.arm_and_enter_offboard()

        if not ok:
            self.get_logger().error("Setup failed. Exiting.")
            return

        await self.send_velocity_loop()


def spin_ros(node):
    rclpy.spin(node)


async def main_async():
    rclpy.init()
    node = CmdVelToPX4()

    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    try:
        await node.run_async()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
