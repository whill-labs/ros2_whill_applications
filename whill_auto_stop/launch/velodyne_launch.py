import os

import yaml

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    whill_auto_stop_share_dir = get_package_share_directory(
        'whill_auto_stop')

    driver_params_file = os.path.join(
        whill_auto_stop_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[driver_params_file]
    )

    convert_share_dir = get_package_share_directory(
        'velodyne_pointcloud')
    convert_params_file = os.path.join(
        whill_auto_stop_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(
        convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[convert_params]
    )

    laserscan_share_dir = get_package_share_directory(
        'velodyne_laserscan')
    laserscan_params_file = os.path.join(
        laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='both',
        parameters=[laserscan_params_file]
    )

    display_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('whill_description'), 'launch',
                             'description_launch.py'),
            ]
        ),
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.18', '0', '0.35', '0', '3.14', '3.14', 'base_link', 'velodyne'],
    )

    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velodyne_driver_node,
            on_exit=[EmitEvent(
                event=launch.events.Shutdown())],
        )
    )

    ld = LaunchDescription()

    ld.add_action(velodyne_driver_node)
    ld.add_action(velodyne_transform_node)
    ld.add_action(velodyne_laserscan_node)

    ld.add_action(display_robot_launch)
    ld.add_action(static_transform_publisher_node)

    ld.add_action(event_handler)

    return ld
