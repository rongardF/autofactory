
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    tool_sn = LaunchConfiguration("tool_sn", default="true")
    tcp_frame_id = LaunchConfiguration("tcp_frame_id", default="tool0")
    tcp = LaunchConfiguration("tcp", default="[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797]")
    simulated = LaunchConfiguration("simulated", default="true")
    mounted = LaunchConfiguration("mounted", default="false")
    flow_rate = LaunchConfiguration("flow_rate", default="1.0")

    declared_arguments = [
        DeclareLaunchArgument("tool_sn", default_value="true", description="Tool serial number."),
        DeclareLaunchArgument("tcp_frame_id", default_value="tool0", description="TCP frame ID."),
        DeclareLaunchArgument("tcp", default_value="[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797]", description="TCP transform as [x, y, z, roll, pitch, yaw] (m, rad)."),
        DeclareLaunchArgument("simulated", default_value="true", description="Whether the tool is simulated."),
        DeclareLaunchArgument("mounted", default_value="false", description="Whether the tool is mounted."),
        DeclareLaunchArgument("flow_rate", default_value="1.0", description="Volumetric flow rate (ml/s). Must be >= 0."),
    ]

    endtool_node = Node(
        package="endtools",
        executable="volumetric_dispensing_tool",
        output="screen",
        parameters=[
            {
                "use_sim_time": simulated,
                "tool_sn": tool_sn,
                "tcp_frame_id": tcp_frame_id,
                "tcp": tcp,
                "simulated": simulated,
                "mounted": mounted,
                "flow_rate": flow_rate,
            },
        ],
    )

    return LaunchDescription(declared_arguments + [endtool_node])