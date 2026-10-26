# 更改为显式加载控制器；轨迹控制器默认激活，速度与直接位置控制器默认不激活。
# 原本的spawn_controllers.launch.py是直接加载moveit_config的ros2_controller的所有控制器，会导致速度和位置控制器同时启动，实际上同时最好只启动一个控制器，避免冲突。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panther_arm_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panther_velocity_controller",
                "--controller-manager",
                "/controller_manager",
                "--inactive",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panther_position_controller",
                "--controller-manager",
                "/controller_manager",
                "--inactive",
            ],
        ),
    ])
