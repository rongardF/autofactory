"""Testbench for the ``manufacturing_executor`` node, fully simulated / local.

Brings up, in one launch, the station of the ``station`` package as far as the
manufacturing loop needs it:

- Gazebo (``world/testbench.world``, world ``default``), ``move_group``, RViz and the
  ``movement_controller`` through the ``movement_controller`` launch, with the testbench scene
  ``urdf/station.urdf.xacro`` (station, UR robot, camera, tool rack and mount, plus the
  product_carrier conveyor and the RFID scanner) and ``srdf/station.srdf.xacro``;
- ``gazebo_world_manager`` and ``moveit2_world_manager`` (``world_manager``);
- the simulated ``basler_camera`` (``cameras``);
- ``station_cache`` and ``object_model_storage`` (local), ``operation_repository`` and
  ``image_repository`` (``repositories``), ``production_manager`` (local assembly file);
- ``tool_mount``, ``tool_rack``, ``tools_manager``, the endtools, the pattern executors and the
  ``dispensing_optimizer`` (``tools_manager`` launch) with the simulated tool inventory of
  ``config/tools_manager_config.yaml``;
- ``conveyor`` (``product_carrier``), the RFID scanner and ``workplace_rfid_scanner``
  (``station_workplace``);
- ``manufacturing_executor`` and ``mock_localizer`` (``manufacturing_executor`` launch).

Not started: the laser sensors and the GPIO controller of the station (not used by the loop).

All lifecycle nodes start Unconfigured; drive them (and the scenario) from ``testbench.ipynb``.
The local-mode files must be in ``download_path`` beforehand; ``setup.sh`` copies them and runs
this launch::

    ./setup.sh
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent
# Must match the world name in world/testbench.world and the workplace simulation.world_name.
WORLD_NAME = 'default'


def _include(package: str, launch_file: str, arguments: dict) -> GroupAction:
    """Include a package launch file, scoped so its declared arguments do not leak."""
    return GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
                ),
                launch_arguments=arguments.items(),
            )
        ],
        scoped=True,
    )


def _lifecycle_node(package: str, executable: str, parameters: dict) -> GroupAction:
    """A lifecycle node of ``package`` named after its executable, started Unconfigured."""
    return GroupAction(
        [
            LifecycleNode(
                package=package,
                executable=executable,
                name=executable,
                namespace='',
                output='screen',
                parameters=[{**parameters, 'use_sim_time': True}],
            )
        ],
        scoped=True,
    )


def generate_launch_description() -> LaunchDescription:
    """Declare the testbench arguments and start the station."""
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    station_id = LaunchConfiguration('station_id')
    download_path = LaunchConfiguration('download_path')
    assembly_file = PathJoinSubstitution([download_path, 'mock_assembly_file.json'])

    declared_arguments = [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Start the Gazebo GUI (false = server only).'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Start RViz.'),
        DeclareLaunchArgument('station_id', default_value='c330289c-0db3-4b5b-9fe0-3d9b63bedb72',
                              description='Station UUID4 (repositories, production_manager).'),
        DeclareLaunchArgument('download_path', default_value='/tmp/station',
                              description='Folder holding local_storage.json, the model entries '
                              'and mock_assembly_file.json.'),
    ]

    movement_controller = _include('movement_controller', 'launch.py', {
        'model': 'ur10',
        'simulated': 'true',
        'debug': 'false',
        'rviz': rviz,
        'urdf_file': str(HERE / 'urdf' / 'station.urdf.xacro'),
        'srdf_file': str(HERE / 'srdf' / 'station.srdf.xacro'),
        'world_file': str(HERE / 'world' / 'testbench.world'),
        'gz_resource_path': PathJoinSubstitution([FindPackageShare('station'), 'config', 'model']),
        'gazebo_gui': gui,
    })
    world_manager = _include('world_manager', 'world_manager.launch.py', {
        'simulated': 'true',
        'world_name': WORLD_NAME,
    })
    cameras = _include('cameras', 'cameras.launch.py', {'simulated': 'true'})
    tools_manager = _include('tools_manager', 'tools_manager_launch.py', {
        'simulated': 'true',
        'world_name': WORLD_NAME,
        'start_world_manager': 'false',
        'station_model_name': 'station',
        'movement_controller_node_name': 'movement_controller',
        'tools_manager_config_file': str(HERE / 'config' / 'tools_manager_config.yaml'),
    })
    production_manager = _include('production_manager', 'production_manager_launch.py', {
        'local_manager': 'true',
        'assembly_file_path': assembly_file,
        'station_id': station_id,
        'use_sim_time': 'true',
    })
    conveyor = _include('product_carrier', 'conveyor.launch.py', {'simulated': 'true'})
    # The workplace launch also starts the RFID scanner (simulated, node simatic_rf240r_ascii).
    workplace = _include('station_workplace', 'workplace_rfid_scanner.launch.py', {
        'use_sim_time': 'true',
        'simulated': 'true',
        'simulation.world_name': WORLD_NAME,
        'start_world_manager': 'false',
    })
    manufacturing_executor = _include('manufacturing_executor', 'manufacturing_executor.launch.py', {
        'use_sim_time': 'true',
    })

    station_cache = _lifecycle_node('repositories', 'station_cache', {
        'station_id': station_id,
        'local_cache': True,
    })
    object_model_storage = _lifecycle_node('repositories', 'object_model_storage', {
        'station_id': station_id,
        'local_storage': True,
        'download_path': download_path,
    })
    operation_repository = _lifecycle_node('repositories', 'operation_repository', {
        'station_id': station_id,
        'assembly_file_path': assembly_file,
    })
    image_repository = _lifecycle_node('repositories', 'image_repository', {
        'station_id': station_id,
    })

    return LaunchDescription([
        *declared_arguments,
        movement_controller,
        world_manager,
        cameras,
        tools_manager,
        station_cache,
        object_model_storage,
        operation_repository,
        image_repository,
        production_manager,
        conveyor,
        workplace,
        manufacturing_executor,
    ])
