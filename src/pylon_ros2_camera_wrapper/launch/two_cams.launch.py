#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context: LaunchContext):
    
    # adapt if needed
    debug = False

    # launch configuration variables
    first_cam_node_name = LaunchConfiguration('first_cam_node_name')
    second_cam_node_name = LaunchConfiguration('second_cam_node_name')

    first_cam_id = LaunchConfiguration('first_cam_id')
    second_cam_id = LaunchConfiguration('second_cam_id')

    first_cam_config_file = LaunchConfiguration('first_cam_config_file')
    second_cam_config_file = LaunchConfiguration('second_cam_config_file')

    mtu_size = LaunchConfiguration('mtu_size')
    startup_user_set = LaunchConfiguration('startup_user_set')
    enable_status_publisher = LaunchConfiguration('enable_status_publisher')
    enable_current_params_publisher = LaunchConfiguration('enable_current_params_publisher')

    respawn = LaunchConfiguration('respawn')
    respawn_str = respawn.perform(context)
    respawn_bool = respawn_str.lower() == 'true'

    # log format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # see https://navigation.ros.org/tutorials/docs/get_backtrace.html
    if debug:
        launch_prefix = ['xterm -e gdb -ex run --args']
    else:
        launch_prefix = ''

    return [
            Node(
                package='pylon_ros2_camera_wrapper',
                namespace=first_cam_id,
                executable='pylon_ros2_camera_wrapper',
                name=first_cam_node_name,
                output='screen',
                respawn=respawn_bool,
                emulate_tty=True,
                prefix=launch_prefix,
                parameters=[
                    first_cam_config_file,
                    {
                        'gige/mtu_size': mtu_size,
                        'startup_user_set': startup_user_set,
                        'enable_status_publisher': enable_status_publisher,
                        'enable_current_params_publisher': enable_current_params_publisher
                    }
                ]
            ),
            Node(
                package='pylon_ros2_camera_wrapper',
                namespace=second_cam_id,
                executable='pylon_ros2_camera_wrapper',
                name=second_cam_node_name,
                output='screen',
                respawn=respawn_bool,
                emulate_tty=True,
                prefix=launch_prefix,
                parameters=[
                    second_cam_config_file,
                    {
                        'gige/mtu_size': mtu_size,
                        'startup_user_set': startup_user_set,
                        'enable_status_publisher': enable_status_publisher,
                        'enable_current_params_publisher': enable_current_params_publisher
                    }
                ]
            ),
        ]

def generate_launch_description():

    # specify your first camera config file name here
    first_cam_config_file = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'config',
        'my_first_cam.yaml'
    )

    # specify your second camera config file name here
    second_cam_config_file = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'config',
        'my_second_cam.yaml'
    )

    # specify your first camera node name here
    declare_first_cam_node_name_cmd = DeclareLaunchArgument(
        'first_cam_node_name',
        default_value='first_cam_node',
        description='Name of the wrapper node.'
    )

    # specify your second camera node name here
    declare_second_cam_node_name_cmd = DeclareLaunchArgument(
        'second_cam_node_name',
        default_value='second_cam_node',
        description='Name of the wrapper node.'
    )

    # specify your first camera id here
    declare_first_cam_id_cmd = DeclareLaunchArgument(
        'first_cam_id',
        default_value='first_cam_id',
        description='Id of the camera. Used as node namespace.'
    )

    # specify your second camera id here
    declare_second_cam_id_cmd = DeclareLaunchArgument(
        'second_cam_id',
        default_value='second_cam_id',
        description='Id of the camera. Used as node namespace.'
    )

    declare_first_cam_config_file_cmd = DeclareLaunchArgument(
        'first_cam_config_file',
        default_value=first_cam_config_file,
        description='Camera parameters structured in a .yaml file.'
    )

    declare_second_cam_config_file_cmd = DeclareLaunchArgument(
        'second_cam_config_file',
        default_value=second_cam_config_file,
        description='Camera parameters structured in a .yaml file.'
    )

    declare_mtu_size_cmd = DeclareLaunchArgument(
        'mtu_size',
        default_value='1500',
        description='Maximum transfer unit size. To enable jumbo frames, set it to a high value (8192 recommended)'
    )

    declare_startup_user_set_cmd = DeclareLaunchArgument(
        'startup_user_set',
        # possible value: Default, UserSet1, UserSet2, UserSet3, CurrentSetting
        default_value='CurrentSetting',
        description='Specific user set defining user parameters to run the camera.'
    )

    declare_enable_status_publisher_cmd = DeclareLaunchArgument(
        'enable_status_publisher',
        default_value='true',
        description='Enable/Disable the status publishing.'
    )

    declare_enable_current_params_publisher_cmd = DeclareLaunchArgument(
        'enable_current_params_publisher',
        default_value='true',
        description='Enable/Disable the current parameter publishing.'
    )

    declare_respawn_cmd = DeclareLaunchArgument(
        'respawn',
        default_value='false',
        description='If true, the node will be respawned if it exits.'
    )

    # Define LaunchDescription variable and return it
    ld_mono = LaunchDescription()
    ld_mono.add_action(declare_first_cam_node_name_cmd)
    ld_mono.add_action(declare_second_cam_node_name_cmd)
    ld_mono.add_action(declare_first_cam_id_cmd)
    ld_mono.add_action(declare_second_cam_id_cmd)
    ld_mono.add_action(declare_first_cam_config_file_cmd)
    ld_mono.add_action(declare_second_cam_config_file_cmd)
    ld_mono.add_action(declare_mtu_size_cmd)
    ld_mono.add_action(declare_startup_user_set_cmd)
    ld_mono.add_action(declare_enable_status_publisher_cmd)
    ld_mono.add_action(declare_enable_current_params_publisher_cmd)
    ld_mono.add_action(declare_respawn_cmd)
    ld_mono.add_action(OpaqueFunction(function=_launch_node))

    return ld_mono
