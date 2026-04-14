from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    description_pkg = FindPackageShare("my_robot_description")
    bringup_pkg = FindPackageShare("my_robot_bringup")
    lvs_pkg = FindPackageShare("lvs_driver")

    xacro_file = PathJoinSubstitution([description_pkg, "urdf", "my_robot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([bringup_pkg, "config", "my_robot_controllers.yaml"])
    lvs_launch_file = PathJoinSubstitution([lvs_pkg, "launch", "lvs.launch.py"])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml, {"robot_description": robot_description}],
        output="screen",
    )

    load_and_activate = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2", "control", "load_controller", "--set-state", "active",
                                "--controller-manager", "/controller_manager", "joint_state_broadcaster"
                            ],
                            output="screen",
                        ),
                        ExecuteProcess(
                            cmd=[
                                "ros2", "control", "load_controller", "--set-state", "active",
                                "--controller-manager", "/controller_manager", "diff_drive_controller"
                            ],
                            output="screen",
                        ),
                        ExecuteProcess(
                            cmd=[
                                "ros2", "control", "load_controller", "--set-state", "active",
                                "--controller-manager", "/controller_manager", "x_axis_controller"
                            ],
                            output="screen",
                        ),
                        ExecuteProcess(
                            cmd=[
                                "ros2", "control", "load_controller", "--set-state", "active",
                                "--controller-manager", "/controller_manager", "y_axis_controller"
                            ],
                            output="screen",
                        ),
                        ExecuteProcess(
                            cmd=[
                                "ros2", "control", "load_controller", "--set-state", "active",
                                "--controller-manager", "/controller_manager", "z_axis_controller"
                            ],
                            output="screen",
                        ),
                    ],
                )
            ],
        )
    )

    # Combined GUI:
    # - diff drive teleop
    # - X/Y/Z joint control
    # - LVS profile viewer
    #
    # IMPORTANT:
    # Change executable name below if your installed script name is different.
    combined_teleop_gui = Node(
        package="fastech_hardware",
        executable="fastech_teleop_gui.py",
        output="screen",
        parameters=[{
            # ---------------------------
            # Diff drive parameters
            # ---------------------------
            "cmd_topic": "/diff_drive_controller/cmd_vel",
            "max_speed": 0.20,
            "max_yaw_rate": 1.0,
            "publish_rate_hz": 20.0,
            "speed_step": 0.0001,
            "yaw_step": 0.05,

            # ---------------------------
            # XYZ joint control parameters
            # ---------------------------
            "use_individual_axis_topics": True,
            "joint_cmd_topic": "/position_controller/commands",
            "x_cmd_topic": "/x_axis_controller/commands",
            "y_cmd_topic": "/y_axis_controller/commands",
            "z_cmd_topic": "/z_axis_controller/commands",
            "line_error_topic": "/x_line_error_mm",

            # ---------------------------
            # XYZ limits
            # ---------------------------
            "x_min": -0.50,
            "x_max": 0.50,
            "y_min": -0.20,
            "y_max": 0.20,
            "z_min": -6.283,
            "z_max": 6.283,

            # ---------------------------
            # XYZ jog defaults
            # ---------------------------
            "x_step_default_mm": 5.0,
            "y_step_default_mm": 1.0,
            "z_step_default_deg": 5.0,

            # ---------------------------
            # LVS
            # ---------------------------
            "lvs_topic": "/lvs/profile",
        }],
    )

    start_teleop = TimerAction(period=5.0, actions=[combined_teleop_gui])

    lvs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lvs_launch_file)
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        robot_state_publisher_node,
        ros2_control_node,
        load_and_activate,
        start_teleop,
        lvs_launch,
    ])