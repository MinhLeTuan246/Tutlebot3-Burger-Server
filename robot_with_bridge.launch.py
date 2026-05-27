from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rosbridge_xml = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )

    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(rosbridge_xml)
    )

    robot_launcher_node = Node(
        package="robot_launcher",
        executable="robot_launcher",
        name="robot_launcher",
        output="screen",
    )

    return LaunchDescription([
        rosbridge_launch,
        robot_launcher_node,
    ])
