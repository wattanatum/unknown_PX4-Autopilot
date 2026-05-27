from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")
    pose_index = LaunchConfiguration("pose_index")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="outdoor_slam_test",
        description="Gazebo world name"
    )

    declare_pose_index = DeclareLaunchArgument(
        "pose_index",
        default_value="0",
        description="Index of robot pose inside /dynamic_pose/info"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    ros_gz_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",

            [
                "/world/", world_name,
                "/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
                "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
            ],

            [
                "/world/", world_name,
                "/dynamic_pose/info"
                "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
            ],

            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen"
    )

    dynamic_pose_to_odom = Node(
        package="unknown_px4_mapping",
        executable="dynamic_pose_to_odom",
        name="dynamic_pose_to_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pose_topic": ["/world/", world_name, "/dynamic_pose/info"],
                "pose_index": pose_index,
            }
        ]
    )

    base_link_to_lidar_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_link_tf",
        output="screen",
        arguments=[
            "0.12", "0", "0.02",
            "0", "0", "0",
            "base_link",
            "link"
        ]
    )

    return LaunchDescription([
        declare_world_name,
        declare_pose_index,
        declare_use_sim_time,

        ros_gz_bridge,
        dynamic_pose_to_odom,
        base_link_to_lidar_link_tf,
    ])
