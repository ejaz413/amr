from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("lvs_driver")
    param_file = os.path.join(pkg_dir, "config", "lvs.yaml")

    return LaunchDescription([

        # ================================
        # LVS Driver Node (C++)
        # ================================
        Node(
            package="lvs_driver",
            executable="lvs_node",
            name="lvs_node",
            output="screen",
            parameters=[param_file],
        ),
        Node(
            package="lvs_driver",
            executable="lvs_pipeline_filter_node.py",
            name="lvs_pipeline_filter_node",
            output="screen",
            parameters=[
                {
                    "input_topic": "/lvs/profile",
                    "filtered_topic": "/lvs/profile_filtered",
                    "corners_topic": "/lvs/corners",
                    "line_thickness_mm": 1.0,
                    "ransac_iterations": 70,
                    "segment_gap_mm": 5.0,
                    "corner_filter_window_size": 60,
                }
            ],
        ),
                # ================================
        # Profile Viewer (Python + OpenCV)
        # ================================
        # Node(
        #     package="lvs_driver",
        #     executable="lvs_profile_viewer.py",
        #     name="lvs_profile_viewer",
        #     output="screen",
        #     parameters=[
        #         {
        #             "topic_name": "/lvs/profile",
        #             "window_name": "LVS Profile Viewer",
        #             "width": 1000,
        #             "height": 600,
        #             "margin": 50,
        #             "show_intensity": True,
        #             "flip_vertical": True,
        #         }
        #     ],
        # ),

    ])