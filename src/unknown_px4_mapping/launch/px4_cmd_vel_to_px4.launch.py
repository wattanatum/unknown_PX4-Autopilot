from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    cmd_vel_to_px4 = ExecuteProcess(
        cmd=[
            "bash", "-c",
            """
            source ~/unknown_px4_ws/mavsdk_ros_env/bin/activate && \
            python3 ~/unknown_px4_ws/src/unknown_px4_mapping/unknown_px4_mapping/cmd_vel_to_px4.py \
              --ros-args \
              -p use_sim_time:=true \
              -p auto_arm:=true \
              -p target_altitude:=1.5 \
              -p altitude_kp:=0.18 \
              -p max_vertical_speed:=0.10 \
              -p max_forward_speed:=0.22 \
              -p max_backward_speed:=0.00 \
              -p min_forward_speed:=0.00 \
              -p max_yaw_speed_deg:=20.0 \
              -p min_yaw_speed_deg:=0.0 \
              -p yaw_deadband_deg:=0.5 \
              -p invert_yaw:=true \
              -p smoothing_alpha:=0.20 \
              -p cmd_vel_topic:=/cmd_vel \
              -p path_topic:=/plan \
              -p odom_topic:=/odom \
              -p reset_on_new_path:=false \
              -p reset_if_stuck_after_new_path:=true \
              -p stuck_check_duration:=3.0 \
              -p stuck_movement_threshold:=0.03 \
              -p stuck_cmd_threshold:=0.005 \
              -p stuck_reset_duration:=0.50 \
              -p stuck_reset_cooldown:=5.0
            """
        ],
        output="screen",
    )

    return LaunchDescription([
        cmd_vel_to_px4,
    ])
