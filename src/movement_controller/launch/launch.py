# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Ron Freimann

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, EventHandler
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


ROBOT_TYPES = {
    "ur": ["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30"]
}


def declare_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "model",
            description=(
                "Robot model being used (e.g. 'ur10e'). The vendor/family is "
                "derived from this value."
            ),
            choices=[
                model for models in ROBOT_TYPES.values() for model in models
            ],
            default_value="ur10",
        ),
        DeclareLaunchArgument(
            "ip_address",
            default_value="192.168.1.9",
            description="IP address of the robot controller (used for real hardware).",
        ),
        DeclareLaunchArgument(
            "simulated",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Run in Gazebo simulation (true) or on real hardware (false). "
                "Selects which vendor launch file is included."
            ),
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Launch in debug mode with verbose logging.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("movement_controller"), "world", "default.world"]
            ),
            description=(
                "Gazebo world file, relative to the package share directory. "
                "Only used when 'simulated' is true."
            ),
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("movement_controller"), "urdf", "ur", "gz_sim.urdf.xacro"]
            ),
            description=(
                "Path to the URDF (xacro) file used to build the robot "
                "description, relative to the package share directory."
            ),
        ),
        DeclareLaunchArgument(
            "srdf_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("movement_controller"), "srdf", "ur", "ur.srdf.xacro"]
            ),
            description=(
                "Path to the SRDF (xacro) file used to build the robot "
                "description semantic, relative to the package share directory."
            ),
        ),
        DeclareLaunchArgument(
            "gz_resource_path",
            default_value="",
            description=(
                "Path to the Gazebo resource files."
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    ]


def get_robot_family(model_value: str) -> str:
    """Return the ROBOT_TYPES family key that the given model belongs to.

    Raises ValueError if the model is not part of any known family.
    """
    for family, models in ROBOT_TYPES.items():
        if model_value in models:
            return family
    raise ValueError(
        f"Model '{model_value}' does not map to any known robot family in ROBOT_TYPES."
    )


def build_moveit_config(family: str, model_value: str, srdf_file: str):
    """Build the MoveIt configuration for the given robot family."""
    if family == "ur":
        return (
            MoveItConfigsBuilder(robot_name=f"{model_value}_robot", package_name="movement_controller")
            .robot_description_semantic(
                srdf_file, {"name": model_value}
            )
            .pilz_cartesian_limits()
            .planning_pipelines(
                default_planning_pipeline="pilz_industrial_motion_planner",
                pipelines=["pilz_industrial_motion_planner", "ompl"],
            )
            .to_moveit_configs()
        )

    # Extend here with additional robot families, e.g.:
    # if family == "fanuc":
    #     return (
    #         MoveItConfigsBuilder(robot_name="fanuc", package_name="movement_controller")
    #         .robot_description_semantic(str(Path("srdf") / "fanuc.srdf.xacro"), {"name": model_value})
    #         .to_moveit_configs()
    #     )

    raise ValueError(f"No MoveIt configuration defined for robot family '{family}'.")


def _build_ur_launch_arguments(
    *, model_value: str, ip_address, world_file, urdf_file, is_simulated: bool, gz_resource_path
) -> dict:
    """Map the generic dispatcher arguments to the UR vendor launch arguments."""
    arguments = {
        "ur_type": model_value,
        "robot_ip": ip_address,
        "description_file": urdf_file,
    }
    # 'world_file' is only consumed by the Gazebo simulation launch file.
    if is_simulated:
        gazebo_gui = LaunchConfiguration("gazebo_gui")
        arguments["gazebo_gui"] = gazebo_gui
        arguments["world_file"] = world_file
        arguments["gazebo_sim_resource_path"] = gz_resource_path
    return arguments


# Per-family builders that translate the generic dispatcher arguments into the
# vendor-specific launch arguments expected by that vendor's gz_sim.launch.py /
# hardware.launch.py files. Each vendor may use its own argument names and decide
# which arguments are relevant for simulation vs. real hardware.
#
# Add a new entry here when onboarding a new robot vendor, e.g.:
#   "fanuc": _build_fanuc_launch_arguments,
VENDOR_LAUNCH_ARGUMENT_BUILDERS = {
    "ur": _build_ur_launch_arguments,
}


def build_vendor_launch_arguments(
    family: str, *, model_value: str, ip_address, world_file, urdf_file, is_simulated: bool, gz_resource_path
) -> dict:
    """Return the vendor-specific launch arguments for the given robot family.

    Raises ValueError if the family has no registered argument builder.
    """
    builder = VENDOR_LAUNCH_ARGUMENT_BUILDERS.get(family)
    if builder is None:
        raise ValueError(
            f"No vendor launch-argument mapping defined for robot family '{family}'."
        )
    return builder(
        model_value=model_value,
        ip_address=ip_address,
        world_file=world_file,
        urdf_file=urdf_file,
        is_simulated=is_simulated,
        gz_resource_path=gz_resource_path,
    )


def load_speed_and_acceleration_constraints(family: str) -> dict:
    """Load the speed and acceleration limits for both joints and 
    cartesian motion from config file.
    """
    joint_limits_file = (
        Path(get_package_share_directory("movement_controller"))
        / "config"
        / "joint_limits.yaml"
    )
    if not joint_limits_file.is_file():
        raise FileNotFoundError(
            f"Speed and acceleration constraints file not found: {joint_limits_file}"
        )

    with joint_limits_file.open("r") as f:
        joint_data = yaml.safe_load(f) or {}

    cartesian_limits_file = (
        Path(get_package_share_directory("movement_controller"))
        / "config"
        / "pilz_cartesian_limits.yaml"
    )
    if not cartesian_limits_file.is_file():
        raise FileNotFoundError(
            f"Speed and acceleration constraints file not found: {cartesian_limits_file}"
        )

    with cartesian_limits_file.open("r") as f:
        cartesian_data = yaml.safe_load(f) or {}

    # find the strictest joint velocity and acceleration limits across all joints
    if family == "ur":
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        lowest_velocity_limit = min(
            [
                joint["max_velocity"] 
                for name, joint in joint_data["joint_limits"].items()
                if (name in joint_names and joint.get("has_velocity_limits", False))
            ]
        )
        lowest_acceleration_limit = min(
            [
                joint["max_acceleration"]
                for name, joint in joint_data["joint_limits"].items()
                if (name in joint_names and joint.get("has_acceleration_limits", False))
            ]
        )
    else:
        raise ValueError(f"No joint limits defined for robot family '{family}'.")
    
    # extract cartesian velocity and acceleration limits
    if "cartesian_limits" not in cartesian_data:
        raise KeyError(
            f"Cartesian limits file {cartesian_limits_file} has no 'cartesian_limits'."
        )
    cartesian_limits = cartesian_data["cartesian_limits"]
    max_cartesian_speed = cartesian_limits.get("max_trans_vel", 1.0)
    max_cartesian_acceleration = cartesian_limits.get("max_trans_acc", 2.25)

    return {
        "constraints.max_cartesian_speed": max_cartesian_speed,
        "constraints.max_cartesian_acceleration": max_cartesian_acceleration,
        "constraints.max_joint_speed": lowest_velocity_limit,
        "constraints.max_joint_acceleration": lowest_acceleration_limit,
    }

def load_joint_position_limits(family: str) -> dict:
    """Build the ``robot_description_planning`` joint position-limit parameters
    for move_group from the joint limits config file.

    These per-joint position limits become the state-space (C-space) bounds
    that every planner (OMPL, PILZ) respects globally. move_group enforces them
    for all planning requests, which is how the joint safety envelope is applied.

    The values are read from ``<share>/config/joint_limits.yaml`` (the same file
    MoveIt already consumes for velocity/acceleration limits). Only joints that
    declare ``has_position_limits: true`` with ``min_position``/``max_position``
    are emitted.

    Returns a flat parameter dict keyed by the fully-qualified move_group
    parameter names, e.g.::

        {
            "robot_description_planning.joint_limits.shoulder_pan_joint.has_position_limits": True,
            "robot_description_planning.joint_limits.shoulder_pan_joint.min_position": 2.0,
            "robot_description_planning.joint_limits.shoulder_pan_joint.max_position": 4.5,
            ...
        }

    .. note::
        To source these limits from launch arguments instead of the YAML file
        later, replace only the ``joint_data`` loading below with a read of the
        resolved launch-argument values; the returned dict shape and the
        move_group wiring stay identical.
    """
    if family != "ur":
        raise ValueError(f"No joint position limits defined for robot family '{family}'.")

    joint_limits_file = (
        Path(get_package_share_directory("movement_controller"))
        / "config"
        / "joint_limits.yaml"
    )
    if not joint_limits_file.is_file():
        raise FileNotFoundError(
            f"Joint limits file not found: {joint_limits_file}"
        )

    with joint_limits_file.open("r") as f:
        joint_data = yaml.safe_load(f) or {}

    joints = joint_data.get("joint_limits", {})

    params: dict = {}
    for name, limits in joints.items():
        if not limits.get("has_position_limits", False):
            continue
        if "min_position" not in limits or "max_position" not in limits:
            raise KeyError(
                f"Joint '{name}' declares has_position_limits but is missing "
                f"min_position/max_position in {joint_limits_file}"
            )
        prefix = f"robot_description_planning.joint_limits.{name}"
        params[f"{prefix}.has_position_limits"] = True
        params[f"{prefix}.min_position"] = float(limits["min_position"])
        params[f"{prefix}.max_position"] = float(limits["max_position"])

    return params


def setup_robot_nodes(context, *args, **kwargs):
    """Build the family-specific MoveIt config and the nodes that consume it.

    'model' is only known at runtime, so it is resolved here to select the
    matching robot family and its MoveIt configuration.
    """
    model_value = LaunchConfiguration("model").perform(context)
    simulated_value = LaunchConfiguration("simulated").perform(context)
    srdf_file_value = LaunchConfiguration("srdf_file").perform(context)
    rviz = LaunchConfiguration("rviz")
    debug = LaunchConfiguration("debug")
    ip_address = LaunchConfiguration("ip_address")
    world_file = LaunchConfiguration("world_file")
    urdf_file = LaunchConfiguration("urdf_file")
    gz_resource_path = LaunchConfiguration("gz_resource_path")

    # 'simulated' is a boolean-like string ("true"/"false").
    is_simulated = simulated_value.lower() in ("true", "1", "yes", "on")
    # Gazebo owns the clock in simulation; real hardware uses wall time.
    sim_time_used = is_simulated

    family = get_robot_family(model_value)

    # region: vendor driver include
    # Each vendor family has its own launch sub-directory named "<family>_launch"
    # that always contains a 'gz_sim.launch.py' and a 'hardware.launch.py'.
    # Pick one based on the 'simulated' argument.
    vendor_launch_dir = (
        Path(get_package_share_directory("movement_controller"))
        / "launch"
        / f"{family}_launch"
    )
    driver_launch_file = "gz_sim.launch.py" if is_simulated else "hardware.launch.py"
    driver_launch_path = vendor_launch_dir / driver_launch_file
    if not driver_launch_path.is_file():
        raise FileNotFoundError(
            f"Vendor launch file not found for family '{family}': {driver_launch_path}"
        )
    driver_launch_arguments = build_vendor_launch_arguments(
        family,
        model_value=model_value,
        ip_address=ip_address,
        world_file=world_file,
        urdf_file=urdf_file,
        is_simulated=is_simulated,
        gz_resource_path=gz_resource_path,
    )
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver_launch_path)),
        launch_arguments=driver_launch_arguments.items(),
    )
    # endregion: vendor driver include

    moveit_config = build_moveit_config(family, model_value, srdf_file_value)
    speed_and_acceleration_constraints = load_speed_and_acceleration_constraints(family)
    joint_position_limits = load_joint_position_limits(family)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            joint_position_limits,
            {
                "use_sim_time": sim_time_used,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("movement_controller"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(rviz),
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": sim_time_used,
            },
        ],
    )

    movement_controller = Node(
        package="movement_controller",
        executable="movement_controller",
        name="movement_controller",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            IfElseSubstitution(
                condition=debug,
                if_value="DEBUG",
                else_value="INFO"
            )
        ],
        parameters=[
            speed_and_acceleration_constraints,
            {
                "use_sim_time": sim_time_used,
            },
        ],
    )

    nodes = [move_group_node, rviz_node, movement_controller]

    # Simulation: Gazebo owns the controller_manager and robot_state_publisher,
    # so start the MoveIt nodes directly alongside the sim driver launch.
    # Real hardware: hardware.launch.py emits 'robot_launched' once the driver
    # is ready, and the MoveIt nodes start only then.
    if is_simulated:
        return [robot_driver_launch] + nodes

    return [
        robot_driver_launch,
        RegisterEventHandler(
            EventHandler(
                matcher=lambda event: event.name == "robot_launched",
                entities=nodes,
            )
        ),
    ]


def generate_launch_description():
    # Create the launch description with the declared launch arguments.
    ld = LaunchDescription(declare_arguments())

    # The robot 'model' is only known at runtime, so vendor resolution, the
    # driver include selection (simulated vs. real hardware) and the MoveIt
    # nodes that consume the model-specific config are all built inside an
    # OpaqueFunction once the argument values can be resolved.
    ld.add_action(OpaqueFunction(function=setup_robot_nodes))

    return ld