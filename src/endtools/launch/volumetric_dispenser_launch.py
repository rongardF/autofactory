
"""Launch the volumetric dispenser and optionally its shared world manager."""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the dispenser launch description."""
    simulated = LaunchConfiguration('simulated', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default=simulated)
    world_name = LaunchConfiguration('world_name', default='default')
    tool_sn = LaunchConfiguration('tool_sn', default='unknown')
    model_id = LaunchConfiguration('model_id', default='dispensing_tool')
    pattern_executor = LaunchConfiguration('pattern_executor', default='')
    tcp_frame_id = LaunchConfiguration('tcp_frame_id', default='tool0')
    tcp_uncalibrated = LaunchConfiguration(
        'tcp_uncalibrated',
        default=(
            '[0.0, -0.267, 0.1315, -1.5707963267948966, '
            '-3.141592653589793, 0.0]'
        ),
    )
    station_cache_name = LaunchConfiguration(
        'station_cache_name', default='station_cache'
    )
    tcp_valid_period = LaunchConfiguration(
        'tcp_valid_period', default='86400.0'
    )
    mounted = LaunchConfiguration('mounted', default='false')
    flowrate = LaunchConfiguration('flowrate', default='1.0')
    localized_product_frame_id = LaunchConfiguration(
        'localized_product_frame_id', default='product_origin'
    )
    station_workplace_node_name = LaunchConfiguration(
        'station_workplace_node_name', default='workplace_rfid_scanner'
    )
    node_name = LaunchConfiguration(
        'node_name', default='volumetric_dispensing_tool'
    )
    start_world_manager = LaunchConfiguration(
        'start_world_manager', default='false'
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'simulated',
            default_value='true',
            description='Whether the tool is simulated.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=simulated,
            description='Whether the tool node uses simulation time.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='default',
            description='Name of the simulation world.',
        ),
        DeclareLaunchArgument(
            'tool_sn',
            default_value='unknown',
            description='Tool serial number.',
        ),
        DeclareLaunchArgument(
            'model_id',
            default_value='dispensing_tool',
            description='Model (resource directory name) of the tool, exposed as the read-only "model" parameter.',
        ),
        DeclareLaunchArgument(
            'pattern_executor',
            default_value='',
            description=(
                'Node name of the pattern executor associated with the tool, '
                'exposed as the read-only "pattern_executor" parameter.'
            ),
        ),
        DeclareLaunchArgument(
            'tcp_frame_id',
            default_value='tool0',
            description='TCP frame ID.',
        ),
        DeclareLaunchArgument(
            'tcp_uncalibrated',
            default_value=(
                '[0.0, -0.267, 0.1315, -1.5707963267948966, '
                '-3.141592653589793, 0.0]'
            ),
            description=(
                'Nominal TCP transform as [x, y, z, roll, pitch, yaw] '
                '(m, rad).'
            ),
        ),
        DeclareLaunchArgument(
            'station_cache_name',
            default_value='station_cache',
            description='Node name of the station_cache service provider.',
        ),
        DeclareLaunchArgument(
            'tcp_valid_period',
            default_value='86400.0',
            description='Default validity period in seconds for cached TCP.',
        ),
        DeclareLaunchArgument(
            'mounted',
            default_value='false',
            description='Whether the tool is mounted.',
        ),
        DeclareLaunchArgument(
            'flowrate',
            default_value='1.0',
            description='Volumetric flow rate (ml/s); must be non-negative.',
        ),
        DeclareLaunchArgument(
            'localized_product_frame_id',
            default_value='product_origin',
            description='Frame of the localized product (published by the localization node).',
        ),
        DeclareLaunchArgument(
            'station_workplace_node_name',
            default_value='workplace_rfid_scanner',
            description=(
                'Simulation only: node name of the station workplace publishing the '
                'spawned product model and link on "simulated_product_metadata".'
            ),
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='volumetric_dispensing_tool',
            description='Name of the endtool lifecycle node.',
        ),
        DeclareLaunchArgument(
            'start_world_manager',
            default_value='false',
            description=(
                'Start the shared world manager; disable when a parent launch '
                'already starts it.'
            ),
        ),
    ]

    world_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('world_manager')
            + '/launch/world_manager.launch.py'
        ),
        launch_arguments={
            'simulated': simulated,
            'world_name': world_name,
        }.items(),
        condition=IfCondition(start_world_manager),
    )

    endtool_node = Node(
        package='endtools',
        executable='volumetric_dispensing_tool',
        name=node_name,
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            ["model:='", model_id, "'"],
            '-p',
            ["pattern_executor:='", pattern_executor, "'"],
        ],
        parameters=[
            {
                'simulated': simulated,
                'world_name': world_name,
                'use_sim_time': use_sim_time,
                'tool_sn': ParameterValue(tool_sn, value_type=str),
                'tcp_frame_id': tcp_frame_id,
                'tcp_uncalibrated': tcp_uncalibrated,
                'station_cache_name': station_cache_name,
                'tcp_valid_period': tcp_valid_period,
                'mounted': mounted,
                'flowrate': flowrate,
                'localized_product_frame_id': localized_product_frame_id,
                'simulation.station_workplace_node_name': station_workplace_node_name,
            },
        ],
    )

    return LaunchDescription(
        declared_arguments + [world_manager_launch, endtool_node]
    )
