"""The Go2 EDU for Pashupati: driver, URDF and sensors (src/xtras/README.md, robot package).

    ros2 launch go2_bringup bringup.launch.py namespace:=go2          # what robot.launch.py starts
    ros2 launch go2_bringup bringup.launch.py camera:=true             # with the RealSense

ROBOT_DRIVER=go2_bringup ROBOT_NS=go2 in /etc/pashupati/pashupati.env. Starts no Pashupati node.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('go2_bringup')
CYCLONEDDS_HOME = os.environ.get('CYCLONEDDS_HOME', os.path.expanduser('~/cyclonedds/install'))
# The SDK's Cyclone DDS 0.10 before ROS's: Foxy's libddsc (0.7) lacks its symbols. Only the
# driver gets it; the stack keeps its own RMW libraries.
SDK_ENV = {
    'LD_LIBRARY_PATH': os.pathsep.join(
        p for p in (os.path.join(CYCLONEDDS_HOME, 'lib'), os.environ.get('LD_LIBRARY_PATH')) if p
    )
}


def config(name: str) -> str:
    return os.path.join(SHARE, 'config', name)


def generate_launch_description():
    with open(os.path.join(SHARE, 'urdf', 'go2.urdf')) as f:
        description = f.read()
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='go2',
                              description='namespace of the driver topics and services'),
        DeclareLaunchArgument('driver_config', default_value=config('go2_driver.yaml'),
                              description='go2_driver_node parameters'),
        DeclareLaunchArgument('lidar', default_value='true', description='start the Mid-360'),
        DeclareLaunchArgument('livox_config', default_value=config('mid360.json'),
                              description='Livox network file: host and LiDAR addresses'),
        # false until the D435i is mounted and calibrated against the Mid-360
        DeclareLaunchArgument('camera', default_value='false', description='start the RealSense'),
        Node(package='go2_bringup', executable='go2_driver_node', name='go2_driver_node',
             namespace=arg('namespace'), parameters=[arg('driver_config')], output='screen',
             additional_env=SDK_ENV),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': description}],
             output='screen'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen', condition=IfCondition(arg('lidar')),
             parameters=[{
                 'xfer_format': 1,  # CustomMsg: what FAST-LIO reads
                 'multi_topic': 0,
                 'data_src': 0,
                 'publish_freq': 10.0,
                 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'user_config_path': arg('livox_config'),
                 'cmdline_input_bd_code': 'livox0000000001',
             }]),
        # /camera/camera/color/image_raw (+ /compressed with image_transport_plugins)
        Node(package='realsense2_camera', executable='realsense2_camera_node', name='camera',
             namespace='camera', parameters=[config('realsense.yaml')], output='screen',
             condition=IfCondition(arg('camera'))),
    ])  # fmt: skip
