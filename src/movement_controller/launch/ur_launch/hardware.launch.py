# Copyright (c) 2021 PickNik, Inc.
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
# Author: Denis Stogl

from launch import LaunchDescription, Event
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

class RobotLaunchedEvent(Event):
    name = "robot_launched"


def launch_setup(context):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    # General arguments
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    controllers_file = LaunchConfiguration("controllers_file")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    physical_params_file = LaunchConfiguration("physical_params_file")
    visual_params_file = LaunchConfiguration("visual_params_file")
    joint_limit_params_file = LaunchConfiguration("joint_limit_params_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    blocking_read = LaunchConfiguration("blocking_read")
    simulated = LaunchConfiguration("simulated")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            {
                "hardware_synchronization.expect_blocking_read_write": LaunchConfiguration(
                    "blocking_read"
                ),
                "overruns.print_warnings": NotSubstitution(LaunchConfiguration("blocking_read")),
            },
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
        ],
        output="screen",
    )

    dashboard_client_node = IncludeLaunchDescription(
        condition=IfCondition(NotSubstitution(simulated)),
        launch_description_source=AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_dashboard_client.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
        }.items(),
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(simulated),
        parameters=[
            {"headless_mode": True},
            {"robot_ip": robot_ip},
        ],
    )

    tool_comm_path = PathJoinSubstitution(
        [
            FindPackagePrefix("ur_client_library"),
            "lib",
            "ur_client_library",
            "tool_communication.py",
        ]
    )

    tool_communication_script = ExecuteProcess(
        name="ur_tool_comm",
        condition=IfCondition(use_tool_communication),
        cmd=[
            tool_comm_path,
            robot_ip,
            "--tcp-port",
            tool_tcp_port,
            "--device-name",
            tool_device_name,
        ],
        output="screen",
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(simulated),
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(simulated),
        parameters=[
            {"headless_mode": True},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    trajectory_until_node = Node(
        package="ur_robot_driver",
        executable="trajectory_until_node",
        name="trajectory_until_node",
        output="screen",
        remappings=[
            (
                "/motion_controller/follow_joint_trajectory",
                f"/{initial_joint_controller.perform(context)}/follow_joint_trajectory",
            ),
        ],
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            parameters=[
                {"verify_payload_on_set": NotSubstitution(simulated)},
                ParameterFile(controllers_file, allow_substs=True),
            ],
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]
    controllers_inactive = [
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]
    if activate_joint_controller.perform(context) == "true":
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_mock_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    # region: robot description
    script_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_client_library"),
            "resources",
            "external_control.urscript",
        ]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params_file,
            " ",
            "kinematics_params:=",
            kinematics_params_file,
            " ",
            "physical_params:=",
            physical_params_file,
            " ",
            "visual_params:=",
            visual_params_file,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "verify_robot_model:=",
            "true",
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "headless_mode:=",
            "true",
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_rx_idle_chars:=",
            tool_rx_idle_chars,
            " ",
            "tool_tx_idle_chars:=",
            tool_tx_idle_chars,
            " ",
            "tool_device_name:=",
            tool_device_name,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "reverse_ip:=",
            reverse_ip,
            " ",
            "script_command_port:=",
            script_command_port,
            " ",
            "reverse_port:=",
            reverse_port,
            " ",
            "script_sender_port:=",
            script_sender_port,
            " ",
            "trajectory_port:=",
            trajectory_port,
            " ",
            "blocking_read:=",
            blocking_read,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # endregion: robot description

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    event = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[EmitEvent(event=RobotLaunchedEvent())],
        )
    )

    nodes_to_start = [
        control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_script,
        controller_stopper_node,
        urscript_interface,
        rsp,
        trajectory_until_node,
        wait_robot_description,
        event
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Robot model of the used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("movement_controller"), "urdf", "ur", "hardware.urdf.xacro"]
            ),
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_limit_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "joint_limits.yaml",
                ]
            ),
            description="Config file containing the joint limits (e.g. velocities, positions) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "physical_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "physical_parameters.yaml",
                ]
            ),
            description="Config file containing the physical parameters (e.g. masses, inertia) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "visual_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "visual_parameters.yaml",
                ]
            ),
            description="Config file containing the visual parameters (e.g. meshes) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulated",
            default_value="true",
            description="Robot arm is simulated in Gazebo if true",
        )
    )
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            choices=[
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
                "motion_primitive_forward_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "config",
                    ]
                ),
                "/",
                LaunchConfiguration("ur_type"),
                "_update_rate.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "blocking_read",
            default_value="true",
            description="Block in read() effectively synchronizing the driver with the robot controller.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])