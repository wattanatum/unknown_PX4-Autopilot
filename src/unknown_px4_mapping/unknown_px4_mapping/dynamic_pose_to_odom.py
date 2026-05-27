#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DynamicPoseToOdom(Node):
    def __init__(self):
        super().__init__("dynamic_pose_to_odom")

        self.declare_parameter("pose_topic", "/world/default/dynamic_pose/info")
        self.declare_parameter("pose_index", 1)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.pose_topic = self.get_parameter("pose_topic").value
        self.pose_index = int(self.get_parameter("pose_index").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            TFMessage,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(f"Subscribing: {self.pose_topic}")
        self.get_logger().info(f"Using transform index: {self.pose_index}")
        self.get_logger().info("Index 1 should be base_link from your Gazebo output")
        self.get_logger().info("Publishing /odom and TF odom -> base_link")

    def pose_callback(self, msg: TFMessage):
        if len(msg.transforms) <= self.pose_index:
            self.get_logger().warn(
                f"Pose index {self.pose_index} not available. "
                f"Message has {len(msg.transforms)} transforms."
            )
            return

        transform = msg.transforms[self.pose_index]

        # The bridged Pose_V has stamp 0, so use ROS time.
        # Because use_sim_time is true and /clock is bridged, this becomes simulation time.
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        odom.pose.pose.orientation = transform.transform.rotation

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = transform.transform.translation.x
        tf_msg.transform.translation.y = transform.transform.translation.y
        tf_msg.transform.translation.z = transform.transform.translation.z
        tf_msg.transform.rotation = transform.transform.rotation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicPoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
