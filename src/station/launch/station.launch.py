# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
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

"""Station launch file.

Launches the ``movement_controller`` using the robot cell (station) specific
URDF, SRDF and Gazebo world files that live in this package's ``config``
folder. The station only supplies these scene/description assets and forwards
them as launch arguments to the reusable ``movement_controller`` launch file,
keeping the movement controller decoupled from any particular robot cell.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    station_share = FindPackageShare("station")
    movement_controller_share = FindPackageShare("movement_controller")
    cameras_share = FindPackageShare("cameras")
    laser_sensors_share = FindPackageShare("laser_sensors")
    io_controllers_share = FindPackageShare("io_controllers")

    model = LaunchConfiguration("model")
    simulated = LaunchConfiguration("simulated")
    debug = LaunchConfiguration("debug")
    rviz = LaunchConfiguration("rviz")
    ip_address = LaunchConfiguration("ip_address")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    declared_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value="ur10",
            description="Robot model being used (e.g. 'ur10e').",
        ),
        DeclareLaunchArgument(
            "simulated",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Run in Gazebo simulation (true) or on real hardware (false). "
                "Also selects the default URDF description supplied by this "
                "station."
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
            "ip_address",
            default_value="192.168.1.9",
            description="IP address of the robot controller (used for real hardware).",
        ),
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    ]

    movement_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [movement_controller_share, "launch", "launch.py"]
            )
        ),
        launch_arguments={
            "model": model,
            "simulated": simulated,
            "debug": debug,
            "rviz": rviz,
            "ip_address": ip_address,
            "urdf_file": IfElseSubstitution(
                condition=simulated,
                if_value=PathJoinSubstitution(
                    [station_share, "config", "urdf", "ur", "gz_sim.urdf.xacro"]
                ),
                else_value=PathJoinSubstitution(
                    [station_share, "config", "urdf", "ur", "real.urdf.xacro"]
                ),
            ),
            "srdf_file": PathJoinSubstitution(
                [station_share, "config", "srdf", "default_setup.srdf.xacro"]
            ),
            "world_file": PathJoinSubstitution(
                [station_share, "config", "world", "default.world"]
            ),
            "gz_resource_path": PathJoinSubstitution(
                [station_share, "config", "model"]
            ),
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [cameras_share, "launch", "cameras.launch.py"]
            )
        ),
        launch_arguments={
            "simulated": simulated
        }.items(),
    )

    laser_cross_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [laser_sensors_share, "launch", "captron_orl2.launch.py"]
            )
        ),
        launch_arguments={
            "simulated": simulated
        }.items(),
    )

    gpio_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [io_controllers_share, "launch", "gpio_controller.launch.py"]
            )
        ),
        launch_arguments={
            "simulated": simulated,
            "use_sim_time": simulated,
            "config_file": PathJoinSubstitution(
                [station_share, "config", "ur10_io_config.yaml"]
            ),
        }.items(),
    )

    # use GroupAction to scope the launch files so that their declared arguments don't leak into 
    # the global namespace
    return LaunchDescription(
        declared_arguments
        + [
            GroupAction([camera_launch], scoped=True),
            GroupAction([laser_cross_launch], scoped=True),
            GroupAction([gpio_controller_launch], scoped=True),
            GroupAction([movement_controller_launch], scoped=True),
        ]
    )
