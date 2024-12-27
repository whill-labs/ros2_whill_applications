import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Set true to launch RViz'
    )

    whill_auto_stop_share_dir = get_package_share_directory(
        'whill_auto_stop')

    whill_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('whill_bringup'), 'launch',
                    'whill_launch.py'),
            ]
        ),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('teleop_twist_joy'), 'launch',
                    'teleop-launch.py'),
            ]
        ),
        launch_arguments={
            'joy_config': 'xbox',
            'joy_vel': 'auto_stop/cmd_vel',
        }.items()
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    whill_auto_stop_share_dir, 'launch',
                    'velodyne_launch.py'),
            ]
        ),
    )

    laserscan_params_file = os.path.join(
        whill_auto_stop_share_dir, 'config', 'auto_stop_area.yaml')
    print(laserscan_params_file)
    laserscan_filter_node = Node(
        package='whill_auto_stop',
        executable='laserscan_filter_node',
        output='log',
        namespace='auto_stop',
        parameters=[laserscan_params_file],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    twist_filter_node = Node(
        package='whill_auto_stop',
        executable='twist_filter_node',
        output='both',
        namespace='auto_stop',
        parameters=[{'negative_logic': True}],
        remappings=[
            ('enable', 'scan_in_range'),
            ('output_twist', '/whill/controller/cmd_vel'),
            ('input_twist', 'cmd_vel'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            whill_auto_stop_share_dir, 'rviz', 'debug.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription()

    ld.add_action(declare_arg_rviz)
    ld.add_action(teleop_launch)
    ld.add_action(velodyne_launch)
    ld.add_action(whill_driver_launch)
    ld.add_action(laserscan_filter_node)
    ld.add_action(twist_filter_node)
    ld.add_action(rviz_node)

    return ld
