import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


def _launch_node(context: LaunchContext):
    """Return the action to launch `pylon_ros2_camera_wrapper`.
    This is required to evaluate `respawn` as boolean.
    """
    # launch configuration variables
    debug = LaunchConfiguration('debug')
    debug_str = debug.perform(context) 
    debug_bool = debug_str.lower() == 'true'

    simulated = LaunchConfiguration('simulated')
    simulated_str = simulated.perform(context) 
    simulated_bool = simulated_str.lower() == 'true'

    
    node_name = LaunchConfiguration('node_name')
    camera_id = LaunchConfiguration('camera_id')

    config_file = LaunchConfiguration('config_file')

    respawn = LaunchConfiguration('respawn')
    respawn_str = respawn.perform(context)
    respawn_bool = respawn_str.lower() == 'true'

    # see https://navigation.ros.org/tutorials/docs/get_backtrace.html
    if debug_bool:
        launch_prefix = ['xterm -e gdb -ex run --args']
    else:
        launch_prefix = ''

    if simulated_bool:
        return [
            # gz -> ROS bridge for the simulated camera sensor.
            Node(
                package='ros_gz_bridge',
                namespace=camera_id,
                executable='parameter_bridge',
                name='basler_camera_gz_bridge',
                output='screen',
                respawn=respawn_bool,
                arguments=[
                    '/simulated_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/simulated_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
            ),
            Node(
                package='cameras',
                namespace=camera_id,
                executable='basler_camera',
                name=node_name,
                output='screen',
                respawn=respawn_bool,
                remappings=[
                    ('simulated_camera/image_raw', "/simulated_camera/image_raw"),
                    ('simulated_camera/camera_info', "/simulated_camera/camera_info"),
                ],
                parameters=[
                    {
                        "use_sim_time": True
                    }
                ]
            ),
        ]
    else:
        # we launch an actual node
        return [
            Node(
                package='pylon_ros2_camera_wrapper',
                namespace=camera_id,
                executable='pylon_ros2_camera_wrapper',
                name=node_name,
                output='screen',
                respawn=respawn_bool,
                emulate_tty=True,
                prefix=launch_prefix,
                parameters=[
                    config_file
                ]
            ),
        ]

def generate_launch_description():
    # create launch description
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "simulated",
            default_value="true",
            description="Whether to use a simulated camera or a real one.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "ip_address",
            default_value="192.168.1.80",
            description="IP address of the camera.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "camera_id",
            default_value="cameras",
            description="Camera will define the namespace of the node.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "node_name",
            default_value="basler_camera",
            description="Name of the node.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('cameras'),
                'config',
                'basler',
                'default.yaml'
            ),
            description='Camera parameters structured in a .yaml file.'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "respawn",
            default_value="false",
            description="Enable respawn mode.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Enable debug mode.",
        )
    )

    ld.add_action(OpaqueFunction(function=_launch_node))

    return ld
