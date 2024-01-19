from launch import LaunchDescription as ld
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from pkg_resources import declare_namespace
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction

n = int(input("Run Launch(1) or Test(2):\n"))

    

def generate_launch_description():
    logger = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )
    config_param = os.path.join(
        get_package_share_directory('lci_calibration'),
        'config',
        'lci_calibration.yaml'
    )

    rviz_file = os.path.join(
        get_package_share_directory('lci_calibration'),
        'rviz',
        'rviz.rviz2'
    )
    # osm_file = DeclareLaunchArgument("osm_file", default_value="/root/slam/slam/osm/osm-nav/src/osm_navigation/osm_file/map.osm")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_file]
    )
    if n == 1:
        node = Node(
            name="lci_calibration",
            package="lci_calibration",
            executable="lci_calibration",
            parameters=[config_param],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")]
        )
    elif n == 2:
        node = Node(
            name="lci_calibration",
            package="lci_calibration",
            executable="lci_test",
            parameters=[config_param],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")]
        )
    else:
        print("Wrong option, exit!")
        exit()
    return ld(
        [
            rviz,
            logger,
            node
        ]
    )

