from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def generate_launch_description():
    pkg = get_package_share_directory("fastech_hardware")
    urdf_path = os.path.join(pkg, "urdf", "fastech_one_linear_axis.urdf")
    ctrl_path = os.path.join(pkg, "config", "controllers.yaml")

    robot_description = open(urdf_path, "r").read()

    # Publishes robot_description topic
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ros2_control_node will subscribe to robot_description topic and initialize
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ctrl_path],
        output="screen",
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
    rsp,
    control_node,
    TimerAction(period=2.0, actions=[jsb, pos]),
])
