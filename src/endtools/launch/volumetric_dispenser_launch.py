
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def _bridge_setup(context, *args, **kwargs):
    """Bridge each slot's Gazebo detachable joint attach/detach/state topics and 
    the tool mount's attach/detach/state topic into ROS when simulating.
    """
    if LaunchConfiguration("simulated").perform(context).lower() != 'true':
        return []

    tool_sn = LaunchConfiguration("tool_sn").perform(context)
    tool_mount_child_link = LaunchConfiguration("tool_mount_link").perform(context)
    tool_rack_child_link = LaunchConfiguration("tool_rack_link").perform(context)


    args = [
        f'/{tool_mount_child_link}/{tool_sn}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
        f'/{tool_mount_child_link}/{tool_sn}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        f'/{tool_mount_child_link}/{tool_sn}/state@std_msgs/msg/String[gz.msgs.StringMsg',
        f'/{tool_rack_child_link}/{tool_sn}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
        f'/{tool_rack_child_link}/{tool_sn}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        f'/{tool_rack_child_link}/{tool_sn}/state@std_msgs/msg/String[gz.msgs.StringMsg',
    ]

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=args,
            output='screen'
        )
    ]

def generate_launch_description():
    simulated = LaunchConfiguration("simulated", default="true")
    tool_sn = LaunchConfiguration("tool_sn", default="unknown")
    tcp_frame_id = LaunchConfiguration("tcp_frame_id", default="tool0")
    tcp = LaunchConfiguration("tcp", default="[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797]")
    mounted = LaunchConfiguration("mounted", default="false")
    flow_rate = LaunchConfiguration("flow_rate", default="1.0")
    tool_rack_link = LaunchConfiguration("tool_rack_link")
    tool_mount_link = LaunchConfiguration("tool_mount_link", default="tool_mount_tcp")

    declared_arguments = [
        DeclareLaunchArgument("simulated", default_value="true", description="Whether the tool is simulated."),
        DeclareLaunchArgument("tool_sn", default_value="unknown", description="Tool serial number."),
        DeclareLaunchArgument("tcp_frame_id", default_value="tool0", description="TCP frame ID."),
        DeclareLaunchArgument("tcp", default_value="[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797]", description="TCP transform as [x, y, z, roll, pitch, yaw] (m, rad)."),
        DeclareLaunchArgument("mounted", default_value="false", description="Whether the tool is mounted."),
        DeclareLaunchArgument("flow_rate", default_value="1.0", description="Volumetric flow rate (ml/s). Must be >= 0."),
        DeclareLaunchArgument("tool_rack_link", description="The link name of the tool rack to which the tool is installed."),
        DeclareLaunchArgument("tool_mount_link", default_value="tool_mount_tcp", description="The link name of the tool mount to which the tool is attached."),
    ]

    endtool_node = Node(
        package="endtools",
        executable="volumetric_dispensing_tool",
        output="screen",
        parameters=[
            {
                "simulated": simulated,
                "use_sim_time": simulated,
                "tool_sn": tool_sn,
                "tcp_frame_id": tcp_frame_id,
                "tcp": tcp,
                "mounted": mounted,
                "flow_rate": flow_rate,
            },
        ],
    )

    return LaunchDescription(declared_arguments + [endtool_node] + [OpaqueFunction(function=_bridge_setup)])