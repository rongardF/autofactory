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

import os
from threading import RLock
from uuid import uuid4

from rclpy import init, shutdown
from pydantic import ValidationError
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.parameter import ParameterValue, ParameterType
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Client as ServiceClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import SetBool

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath
from tools_manager.msg import Slots, ToolInfo
from tools_manager.action import MountTool, UnmountTool

from tools_manager.utils.config_reader import read_config_file
from tools_manager.exception.activation_failed_exception import ActivationFailedException
from tools_manager.exception.deactivation_failed_exception import DeactivationFailedException
from tools_manager.exception.cleanup_failed_exception import CleanupFailedException
from tools_manager.exception.world_manager_exception import ModelAttachError
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.slot_frames_dto import SlotFramesDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.endtool_launch_dto import EndtoolLaunchDto
from tools_manager.model.tools_manager_node_config import ToolsManagerNodeConfigDTO
from tools_manager.model.tools_manager_config import ToolSlotDTO
from tools_manager.services.gazebo_world_manager import GazeboWorldManager
from tools_manager.services.moveit2_world_manager import Moveit2WorldManager
from tools_manager.services.node_state_manager import NodeStateManager
from tools_manager.services.ros2_launcher import Ros2Launcher

class ToolsManager(LifecycleNode):

    def __init__(self, node_name: str = 'tools_manager') -> None:
        """Initialise the ToolsManager lifecycle node.

        Declares all ROS 2 parameters with defaults and descriptions and
        creates the internal thread-safety primitives used to serialise
        concurrent goal and cancel callbacks.

        :param node_name: ROS 2 node name passed to :class:`rclpy.lifecycle.LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self._gazebo_service: GazeboWorldManager|None = None
        self._planner_service: Moveit2WorldManager|None = None
        self._tool_mount_manager: NodeStateManager|None = None
        self._tool_rack_manager: NodeStateManager|None = None
        self._ros2_launcher: Ros2Launcher|None = None
        self._endtools: dict[str, EndtoolLaunchDto] = {}
        self._config: ToolsManagerNodeConfigDTO|None = None

        self._slots_info: SlotsDto|None = None
        self._slots_info_lock = RLock()

        self._tool_mounted_info: ToolInfoDto|None = None
        self._tool_mounted_info_lock = RLock()

        self._tool_action_lock = RLock()

        # region: parameters
        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(description='Whether the node is running in simulation mode'),
        )
        self.declare_parameter(
            'world_name',
            "world",
            ParameterDescriptor(description='The name of the world to load in Gazebo'),
        )
        self.declare_parameter(
            'station_model_name',
            "station",
            ParameterDescriptor(description='The name of the station model'),
        )
        self.declare_parameter(
            'tool_mount_node_name',
            "/tool_mount",
            ParameterDescriptor(description='Tool mount node name to communicate with'),
        )
        self.declare_parameter(
            'tool_rack_node_name',
            "/tool_rack",
            ParameterDescriptor(description='Tool rack node name to communicate with'),
        )
        self.declare_parameter(
            'movement_controller_node_name',
            "/movement_controller",
            ParameterDescriptor(description='Movement controller node name to communicate with'),
        )
        self.declare_parameter(
            'tools_manager_config_file',
            "tools_manager_config.yaml",
            ParameterDescriptor(description='Path to the tools manager configuration file'),
        )
        
        # subscriptions and publishers
        self._tool_rack_slots_subscription: Subscription|None = None
        self._tool_mounted_subscription: Subscription|None = None

        # service clients
        self._tool_mount_lock: ServiceClient|None = None

        # action servers and clients
        self._mount_tool_action: ActionServer|None = None
        self._unmount_tool_action: ActionServer|None = None

        self._movement_controller_action_client: ActionClient|None = None

        # callback groups
        self._action_callback_group = ReentrantCallbackGroup()
        self._subscription_callback_group = ReentrantCallbackGroup()

    # region: properties
    @property
    def tool_rack_slots(self) -> SlotsDto|None:
        with self._slots_info_lock:
            return self._slots_info

    @tool_rack_slots.setter
    def tool_rack_slots(self, value: SlotsDto) -> None:
        with self._slots_info_lock:
            self._slots_info = value

    @property
    def tool_mounted(self) -> ToolInfoDto|None:
        with self._tool_mounted_info_lock:
            return self._tool_mounted_info

    @tool_mounted.setter
    def tool_mounted(self, value: ToolInfoDto|None) -> None:
        with self._tool_mounted_info_lock:
            self._tool_mounted_info = value

    # endregion: properties

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        try:
            self.get_logger().info('Reading tools manager config file')
            manager_config = read_config_file(self.get_parameter('tools_manager_config_file').get_parameter_value().string_value)
            self._config = ToolsManagerNodeConfigDTO(
                tools_manager_config=manager_config,
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                world_name=self.get_parameter('world_name').get_parameter_value().string_value,
                station_model_name=self.get_parameter('station_model_name').get_parameter_value().string_value,
                tool_mount_node_name=self.get_parameter('tool_mount_node_name').get_parameter_value().string_value,
                tool_rack_node_name=self.get_parameter('tool_rack_node_name').get_parameter_value().string_value,
                movement_controller_node_name=self.get_parameter('movement_controller_node_name').get_parameter_value().string_value,
                tools_manager_config_file=self.get_parameter('tools_manager_config_file').get_parameter_value().string_value,
            )
            self.get_logger().info(f'Tools manager config file read successfully: {self._config}')

            self.get_logger().info('Creating world manager services')
            if self._config.simulated:
                self._gazebo_service = GazeboWorldManager(
                    node=self,
                    world_name=self._config.world_name,
                    station_model_name=self._config.station_model_name,
                    tools_manager_config=self._config.tools_manager_config,
                    tool_mount_link="tool_mount_tcp",
                )
            self._planner_service = Moveit2WorldManager(
                node=self,
                tools_manager_config=self._config.tools_manager_config,
            )

            self.get_logger().info('Creating node state managers and ROS2 launcher')
            self._tool_rack_manager = NodeStateManager(self, self._config.tool_rack_node_name)
            self._tool_mount_manager = NodeStateManager(self, self._config.tool_mount_node_name)
            self._ros2_launcher = Ros2Launcher(self)
        except ValidationError as e:
            self.get_logger().error(f'Failed to read tools manager config file: {e}')
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            self._gazebo_service = None
            self._planner_service = None
            self._tool_mount_manager = None
            self._ros2_launcher = None
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to activate node: super().on_activate() returned FAILURE')
            return TransitionCallbackReturn.FAILURE

        # sanity checks before activating the node
        if (
            self._ros2_launcher is None or
            self._planner_service is None or
            self._config is None or
            self._tool_rack_manager is None or
            self._tool_mount_manager is None
        ):
            self.get_logger().error('Sanity check failed, something is un-initialized. Cannot activate node.')
            return TransitionCallbackReturn.FAILURE

        if (
            self._config.simulated and
            self._gazebo_service is None
        ):
            self.get_logger().error('Gazebo world manager is not initialized.')
            return TransitionCallbackReturn.FAILURE
        
        try:
            # configure and activate tool rack and tool mount
            for manager in (self._tool_rack_manager, self._tool_mount_manager):
                if manager.configure_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to configure node [{manager.node_name}]')
                    return TransitionCallbackReturn.FAILURE
                if manager.activate_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to activate node: {manager.node_name}')
                    return TransitionCallbackReturn.FAILURE

            # create subscription for tool_rack and tool_mouunt topics that provide
            # installed and mounted endtool info
            self._slots_subscription = self.create_subscription(
                msg_type=Slots,
                topic=f'{self._config.tool_rack_node_name}/slots',
                callback=self._slots_callback,
                callback_group=self._subscription_callback_group,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
            self._tool_mounted_subscription = self.create_subscription(
                msg_type=ToolInfo,
                topic=f'{self._config.tool_mount_node_name}/tool_mounted',
                callback=self._mounted_callback,
                callback_group=self._subscription_callback_group,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )

            # get info of tools installed on rack and mounted on tool mount
            if self._config.simulated:
                self.get_logger().info('Running in simulation mode, using config to get tools installed and mounted.')
                tools_installed: dict[int, ToolInfoDto] = {}
                for entry in self._config.tools_manager_config.simulation_setup.tool_rack:
                    if entry.tag_data is not None:
                        tools_installed[entry.index] = ToolInfoDto.from_tag_data(entry.index, entry.tag_data)
                
                if self._config.tools_manager_config.simulation_setup.tool_mount is not None:
                    # index '-1' significes that tool is NOT munted on any of the slots
                    tool_mounted = ToolInfoDto.from_tag_data(
                        -1, self._config.tools_manager_config.simulation_setup.tool_mount,
                    )
                else:
                    tool_mounted = None
            else:
                self.get_logger().info('Running in hardware mode, waiting for tool rack and tool mount to publish installed and mounted tools info.')
                for _ in range(10):
                    slots = self.tool_rack_slots
                    tool_mounted = self.tool_mounted
                    if slots is None or tool_mounted is None:
                        self.get_clock().sleep_for(Duration(seconds=1.0))
                        continue
                    break
                else:
                    raise ActivationFailedException('Failed to get tool rack slots and/or tool mounted info after 10 attempts.')

                tools_installed = {tool.index: tool for tool in slots.tools_mounted}

            # launch endtool nodes (+supporting nodes) sitting on tool_rack and spawn models into
            # planning scene and Gazebo world (if in simulation mode)
            self.get_logger().info('Launching and spawning endtools nodes and models')
            for index, tool in tools_installed.items():
                if tool is None:
                    self.get_logger().warning(f'Slot {index} is empty, no endtool launched.')
                    continue

                tool_parameters = self._config.tools_manager_config.get_tool_parameters(tool.tool_sn)
                tool_metadata = self._config.tools_manager_config.get_tool_metadata(tool.tool_sn)

                if tool_parameters.get("node_name", None) is None or tool_metadata is None:
                    raise ActivationFailedException(
                        f'Failed to get parameters and/or metadata for tool with serial number {tool.tool_sn}. '
                        f'Cannot launch endtool node.'
                    )

                launch_ref = self._ros2_launcher.launch(
                    launch_file=self._get_full_launch_file_path(
                        tool_metadata.launch_file
                    ),
                    launch_arguments={
                        **tool_parameters,
                        "simulated": self._config.simulated,
                        "use_sim_time": self._config.simulated,
                        "tool_sn": tool.tool_sn,
                        "mounted": "false",  # tools on rack are not mounted
                        "tool_rack_link": f'slot{index}_attached_link',
                        "tool_mount_link": "tool_mount_tcp"
                    },
                )
                endtool_node_manager = NodeStateManager(
                    node=self, node_name=str(tool_parameters["node_name"])
                )
                self._endtools[tool.tool_sn] = EndtoolLaunchDto(
                    launch=launch_ref,
                    endtool_node_manager=endtool_node_manager,
                )
                if endtool_node_manager.configure_node() != TransitionCallbackReturn.SUCCESS:
                    raise ActivationFailedException(
                        f'Failed to configure endtool node {tool_parameters["node_name"]!r} '
                        f'for tool {tool.tool_sn}.'
                    )
                
                self._planner_service.spawn_model(tool, link_name=f'slot{index}_attached_link')
    
                if self._config.simulated and self._gazebo_service is not None:
                    self._gazebo_service.spawn_model(tool, link_name=f'slot{index}_attached_link')

            # launch endtool on tool_mount (if any mounted)
            if tool_mounted is not None:
                self.get_logger().info(f'Launching and spawning endtool node and model for mounted tool: {tool_mounted.tool_sn}')
                tool_parameters = self._config.tools_manager_config.get_tool_parameters(tool_mounted.tool_sn)
                tool_metadata = self._config.tools_manager_config.get_tool_metadata(tool_mounted.tool_sn)
                slot_index = self._config.tools_manager_config.get_tool_slot_number(tool_mounted.tool_sn)

                if tool_parameters.get("node_name", None) is None or tool_metadata is None:
                    raise ActivationFailedException(
                        f'Failed to get parameters and/or metadata for tool with serial number {tool_mounted.tool_sn}. '
                        f'Cannot launch endtool node.'
                    )

                launch_ref = self._ros2_launcher.launch(
                    launch_file=self._get_full_launch_file_path(
                        tool_metadata.launch_file,
                    ),
                    launch_arguments={
                        **tool_parameters,
                        "simulated": self._config.simulated, # NOTE: over-writting these parameters 
                        "use_sim_time": self._config.simulated,
                        "tool_sn": tool_mounted.tool_sn,
                        "mounted": "true",
                        "tool_rack_link": f'slot{slot_index}_attached_link',
                        "tool_mount_link": "tool_mount_tcp"
                    },
                )
                endtool_node_manager = NodeStateManager(
                    node=self, node_name=str(tool_parameters["node_name"])
                )
                self._endtools[tool_mounted.tool_sn] = EndtoolLaunchDto(
                    launch=launch_ref,
                    endtool_node_manager=endtool_node_manager,
                )
                if endtool_node_manager.configure_node() != TransitionCallbackReturn.SUCCESS:
                    raise ActivationFailedException(
                        f'Failed to configure endtool node {tool_parameters["node_name"]!r} '
                        f'for tool {tool_mounted.tool_sn}.'
                    )
                
                self._planner_service.spawn_model(tool_mounted, link_name="tool_mount_tcp", rotation=(0.0, 0.0, 1.0, 0.0))
    
                if self._config.simulated and self._gazebo_service is not None:
                    self._gazebo_service.spawn_model(tool_mounted, link_name="tool_mount_tcp", rotation=(0.0, 0.0, 1.0, 0.0))

            self.get_logger().info('Creating interfaces for tools manager')
            # creating tool mount lock service client
            self._tool_mount_lock = self.create_client(
                srv_type=SetBool,
                srv_name=f'{self._config.tool_mount_node_name}/lock',
            )

            # create movement controller action client
            self._movement_controller_action_client = ActionClient(
                self,
                ExecuteTrajectory,
                f'{self._config.movement_controller_node_name}/execute_trajectory',
            )

            # create tool managing action clients
            self._mount_tool_action = ActionServer(
                node=self,
                action_type=MountTool,
                action_name='~/mount_tool',
                execute_callback=self._mount_tool_callback,
                cancel_callback=self._cancel_tool_action_callback,
                callback_group=self._action_callback_group,
            )
            self._unmount_tool_action = ActionServer(
                node=self,
                action_type=UnmountTool,
                action_name='~/unmount_tool',
                execute_callback=self._unmount_tool_callback,
                cancel_callback=self._cancel_tool_action_callback,
                callback_group=self._action_callback_group,
            )
            
        except ActivationFailedException as e:
            self.get_logger().error(f'Activation failed: {e}')
            if self._tool_mounted_subscription is not None:
                self.destroy_subscription(self._tool_mounted_subscription)
                self._tool_mounted_subscription = None
            if self._slots_subscription is not None:
                self.destroy_subscription(self._slots_subscription)
                self._slots_subscription = None
            for manager in (self._tool_rack_manager, self._tool_mount_manager):
                if manager.deactivate_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to deactivate node [{manager.node_name}]')
                    return TransitionCallbackReturn.FAILURE
                if manager.unconfigure_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to unconfigure node: {manager.node_name}')
                    return TransitionCallbackReturn.FAILURE

            if self._endtools:
                for tool_sn, endtool_launch in self._endtools.items():
                    endtool_launch.endtool_node_manager.deactivate_node()
                    self._planner_service.delete_model(tool_sn)
                    if self._config.simulated and self._gazebo_service is not None:
                        self._gazebo_service.delete_model(tool_sn)
                    self._ros2_launcher.shutdown_all()
                self._endtools = {}

            if self._tool_mount_lock is not None:
                self._tool_mount_lock.destroy()
                self._tool_mount_lock = None

            if self._movement_controller_action_client is not None:
                self._movement_controller_action_client.destroy()
                self._movement_controller_action_client = None

            if self._mount_tool_action is not None:
                self._mount_tool_action.destroy()
                self._mount_tool_action = None
            if self._unmount_tool_action is not None:
                self._unmount_tool_action.destroy()
                self._unmount_tool_action = None
            
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to deactivate node: super().on_deactivate() returned FAILURE')
            raise DeactivationFailedException('Failed to deactivate node: super().on_deactivate() returned FAILURE')

        for manager in (self._tool_rack_manager, self._tool_mount_manager):
            if manager is not None:
                if manager.deactivate_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to deactivate node [{manager.node_name}]')
                    raise DeactivationFailedException(f'Failed to deactivate node [{manager.node_name}]')
                if manager.unconfigure_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to unconfigure node: {manager.node_name}')
                    raise DeactivationFailedException(f'Failed to unconfigure node: {manager.node_name}')

        if self._slots_subscription is not None:
            self.destroy_subscription(self._slots_subscription)
            self._slots_subscription = None

        if self._tool_mounted_subscription is not None:
            self.destroy_subscription(self._tool_mounted_subscription)
            self._tool_mounted_subscription = None

        if self._movement_controller_action_client is not None:
            self._movement_controller_action_client.destroy()
            self._movement_controller_action_client = None

        if self._mount_tool_action is not None:
            self._mount_tool_action.destroy()
            self._mount_tool_action = None

        if self._unmount_tool_action is not None:
            self._unmount_tool_action.destroy()
            self._unmount_tool_action = None

        if self._tool_mount_lock is not None:
            self._tool_mount_lock.destroy()
            self._tool_mount_lock = None

        for tool_sn, endtool_launch in self._endtools.items():
            endtool_node_state = endtool_launch.endtool_node_manager.get_node_state()
            if endtool_node_state is not None and endtool_node_state == 'active':
                if endtool_launch.endtool_node_manager.deactivate_node() != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(f'Failed to deactivate endtool node [{endtool_launch.endtool_node_manager.node_name}]')
                    raise DeactivationFailedException(f'Failed to deactivate endtool node [{endtool_launch.endtool_node_manager.node_name}]')
            if endtool_launch.endtool_node_manager.unconfigure_node() != TransitionCallbackReturn.SUCCESS:
                self.get_logger().error(f'Failed to unconfigure endtool node: {endtool_launch.endtool_node_manager.node_name}')
                raise DeactivationFailedException(f'Failed to unconfigure endtool node: {endtool_launch.endtool_node_manager.node_name}')

            if self._planner_service is not None:
                self._planner_service.delete_model(tool_sn)

            if self._config and self._config.simulated and self._gazebo_service is not None:
                self._gazebo_service.delete_model(tool_sn)
        
        if self._ros2_launcher is not None:
            self._ros2_launcher.shutdown_all()

        self._endtools = {}

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to cleanup node: super().on_cleanup() returned FAILURE')
            raise CleanupFailedException('Failed to cleanup node: super().on_cleanup() returned FAILURE')

        self._planner_service = None
        self._gazebo_service = None
        self._ros2_launcher = None
        self._tool_mount_manager = None
        self._tool_rack_manager = None
        self._config = None

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)

    # endregion: lifecycle callbacks

    # region: callbacks
    def _slots_callback(self, msg: Slots) -> None:
        self.tool_rack_slots = SlotsDto.from_slots_msg(msg)

    def _mounted_callback(self, msg: ToolInfo) -> None:
        tool_mounted = ToolInfoDto.from_msg(msg)
        if tool_mounted.tool_sn:
            self.tool_mounted = tool_mounted
        else:
            self.tool_mounted = None

    def _cancel_tool_action_callback(self, goal_handle) -> bool:
        self.get_logger().info(f'Cancel request received for goal: {goal_handle}')
        return True  # TODO: implement cancel logic

    def _mount_tool_callback(self, goal_handle: ServerGoalHandle) -> MountTool.Result:
        request: MountTool.Goal = goal_handle.request
        response = MountTool.Result()

        if self._tool_action_lock.acquire(blocking=False) is False:
            response.success = False
            response.message = "Another mount/unmount operation is in progress. Please try again later."
            goal_handle.abort()
            return response
        
        with self._tool_action_lock:
            self._tool_action_lock.release()  # release the lock as we already hold it
            tool_rack_slots = self.tool_rack_slots
            tool_mounted = self.tool_mounted

            if tool_mounted is not None:
                response.success = False
                response.message = "A tool is already mounted. Please unmount it first."
                goal_handle.abort()
                return response

            if tool_rack_slots is None:
                response.success = False
                response.message = "Tool rack slots info is not available. Cannot perform mount operation."
                goal_handle.abort()
                return response

            # sanity check
            if (
                self._planner_service is None or
                self._gazebo_service is None or
                self._movement_controller_action_client is None or
                self._config is None or 
                self._tool_mount_lock is None
            ):
                response.success = False
                response.message = "Sanity check failed (something is un-initialized). Cannot perform mount operation."
                goal_handle.abort()
                return response

            # TODO: consider how failure should be handled - do we roll back or unconfigure or error?

            tool_sn = request.tool_sn
            tool_info = tool_rack_slots.get_tool_info(tool_sn)
            if tool_info is None:
                response.success = False
                response.message = f"Tool with serial number '{tool_sn}' is not on the rack."
                goal_handle.abort()
                return response

            # get the parent frames for moving with 'tool_mount_tcp' frame/link; all frames are
            # defined in such a way that movement pose required is all zeros - this means that
            # 'tool_mount_tcp' frame must align with the target frame and then we are in correct pose
            result = self._get_frames_and_pose(tool_info)
            if result is None:
                response.success = False
                response.message = f"Tool with serial number '{tool_sn}' is not on the rack."
                goal_handle.abort()
                return response
            frames, unity_pose = result

            # move to tool_slide in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_PTP
            path.tool_frame = 'tool_mount_tcp'  # NOTE: this is hardcoded frame and matches the link defined in URDF; DO NOT CHANGE IT UNLESS CHANGING IN URDF ALSO!
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_slide_in_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_slide_in pose."
                goal_handle.abort()
                return response

            # allow collisions between tool-mount, endtool and tool-rack for the duration of the mount operation
            if not self._planner_service.allow_collisions(model_id=tool_sn, allowed=True, tool_mount_link='tool_mount_tcp', slot_link='tool_rack_link'):   
                response.success = False
                response.message = "Failed to allow collisions between tool-mount, endtool and tool-rack."
                goal_handle.abort()
                return response

            # operate tool-mount quick release to open the lock for mounting the tool
            if self._tool_mount_lock.wait_for_service(timeout_sec=5.0) is False:
                response.success = False
                response.message = "Failed to connect to lock service to unlock tool."
                goal_handle.abort()
                return response
            else:
                lock_request = SetBool.Request(data=False)  # False means unlock, True means lock
                if self._call_service(self._tool_mount_lock, lock_request) is False:
                    response.success = False
                    response.message = "Failed to unlock tool-mount quick release."
                    goal_handle.abort()
                    return response

            # move to tool_attached in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.cartesian_speed = 0.08
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_attached_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_attached pose."
                goal_handle.abort()
                return response

            # operate tool-mount quick release to mount the tool
            if self._tool_mount_lock.wait_for_service(timeout_sec=5.0) is False:
                response.success = False
                response.message = "Failed to connect to lock service to lock tool."
                goal_handle.abort()
                return response
            else:
                lock_request = SetBool.Request(data=True)  # False means unlock, True means lock
                if self._call_service(self._tool_mount_lock, lock_request) is False:
                    response.success = False
                    response.message = "Failed to lock tool-mount quick release."
                    goal_handle.abort()
                    return response

            # check that tool_mount can detect the tool and its correct tool
            for _ in range(10):  # attempt up to 10 seconds to detect the mounted tool
                new_tool_mounted = self.tool_mounted
                if new_tool_mounted is not None and new_tool_mounted.tool_sn == tool_sn:
                    break
                self.get_clock().sleep_for(Duration(seconds=1.0))
            else:
                response.success = False
                response.message = f"Tool-mount failed to detect the mounted tool with serial number '{tool_sn}'."
                goal_handle.abort()
                return response

            # wait for a bit to ensure that the tool is fully mounted and detected (otherwise it will
            # be attached while the tool-mount is moving and tool will be attached with incorrect pose
            self.get_clock().sleep_for(Duration(seconds=0.5))  

            try:
                # detach tool from rack in planning scene and attach to tool-mount
                self._planner_service.attach_to_link(model_id=tool_sn, link_name='tool_mount_tcp')

                # if simulating then transfer model also in Gazebo
                if self._config.simulated:
                    self._gazebo_service.attach_to_link(model_id=tool_sn, link_name='tool_mount_tcp')
            except ModelAttachError as e:
                response.success = False
                response.message = f"Failed to attach tool to tool-mount in planning scene: {e}"
                goal_handle.abort()
                return response

            # move to tool_lifted pose with 'tool_mount_tcp'
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.cartesian_speed = 0.08
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_lifted_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_lifted pose."
                goal_handle.abort()
                return response

            # re-enable collisions between tool-mount and tool-rack after the mount operation
            self._planner_service.allow_collisions(model_id=tool_sn, allowed=False, tool_mount_link='tool_mount_tcp', slot_link='tool_rack_link')

            # reconfigure endtool as mounted
            self._endtool_mounted(tool_sn, True)

            response.success = True
            goal_handle.succeed()
            return response

    def _unmount_tool_callback(self, goal_handle: ServerGoalHandle) -> UnmountTool.Result:
        request: UnmountTool.Goal = goal_handle.request
        response = UnmountTool.Result()

        if self._tool_action_lock.acquire(blocking=False) is False:
            response.success = False
            response.message = "Another mount/unmount operation is in progress."
            goal_handle.abort()
            return response
        
        with self._tool_action_lock:
            self._tool_action_lock.release()  # release the lock as we already hold it
            tool_mounted = self.tool_mounted
            slots = self.tool_rack_slots
            if tool_mounted is None:
                response.success = False
                response.message = "No tool is currently mounted."
                goal_handle.abort()
                return response
            else:
                tool_sn = tool_mounted.tool_sn

            if slots is None:
                response.success = False
                response.message = "Tool rack slots info is not available. Cannot perform unmount operation."
                goal_handle.abort()
                return response
            else:
                slot_info = slots.get_slot_info(tool_sn)
                if slot_info is None:
                    response.success = False
                    response.message = f"Slot information for tool with serial number '{tool_sn}' is not available. Cannot unmount tool."
                    goal_handle.abort()
                    return response
                elif not slots.is_slot_empty(slot_info.index):
                    response.success = False
                    response.message = f"Slot {slot_info.index} is not empty. Cannot unmount tool with serial number '{tool_sn}'."
                    goal_handle.abort()
                    return response
                
            # sanity check
            if (
                self._planner_service is None or
                self._gazebo_service is None or
                self._movement_controller_action_client is None or
                self._config is None or 
                self._tool_mount_lock is None
            ):
                response.success = False
                response.message = "Sanity check failed (something is un-initialized). Cannot perform unmount operation."
                goal_handle.abort()
                return response

            # TODO: consider how failure should be handled - do we roll back or unconfigure or error?

            # get the parent frames for moving with 'tool_mount_tcp' frame/link; all frames are
            # defined in such a way that movement pose required is all zeros - this means that
            # 'tool_mount_tcp' frame must align with the target frame and then we are in correct pose
            result = self._get_frames_and_pose(slot_info)
            if result is None:
                response.success = False
                response.message = f"Tool with serial number '{tool_sn}' is not mounted."
                goal_handle.abort()
                return response
            frames, unity_pose = result

            # move to tool_lifted in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_PTP
            path.tool_frame = 'tool_mount_tcp'  # NOTE: this is hardcoded frame and matches the link defined in URDF; DO NOT CHANGE IT UNLESS CHANGING IN URDF ALSO!
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_lifted_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_lifted pose."
                goal_handle.abort()
                return response

            # allow collisions between tool-mount, endtool and tool-rack for the duration of the mount operation
            if not self._planner_service.allow_collisions(model_id=tool_sn, allowed=True, tool_mount_link='tool_mount_tcp', slot_link='tool_rack_link'):   
                response.success = False
                response.message = "Failed to allow collisions between tool-mount, endtool and tool-rack."
                goal_handle.abort()
                return response

            # move to tool_attached in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.cartesian_speed = 0.08
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_attached_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_attached pose."
                goal_handle.abort()
                return response

            try:
                # detach tool from mount in planning scene and attach to tool-rack slot
                self._planner_service.attach_to_link(model_id=tool_sn, link_name=result[0].tool_attached_frame)

                # if simulating then also transfer in Gazebo
                if self._config.simulated:
                    self._gazebo_service.attach_to_link(tool_sn, link_name=result[0].tool_attached_frame)
            except ModelAttachError as e:
                response.success = False
                response.message = f"Failed to attach tool to tool-rack in planning scene: {e}"
                goal_handle.abort()
                return response

            # operate tool-mount quick release to open the lock for un-mounting the tool
            if self._tool_mount_lock.wait_for_service(timeout_sec=5.0) is False:
                response.success = False
                response.message = "Failed to connect to lock service to unlock tool."
                goal_handle.abort()
                return response
            else:
                lock_request = SetBool.Request(data=False)  # False means unlock, True means lock
                if self._call_service(self._tool_mount_lock, lock_request) is False:
                    response.success = False
                    response.message = "Failed to unlock tool-mount quick release."
                    goal_handle.abort()
                    return response

            # ensure that tool_rack can detect the tool and its correct tool
            for _ in range(10):  # attempt up to 10 seconds to detect the unmounted tool
                new_slots = self.tool_rack_slots
                if new_slots is not None:
                    new_tool_info = new_slots.get_tool_info(tool_sn)
                    if new_tool_info is not None:
                        break
                self.get_clock().sleep_for(Duration(seconds=1.0))
            else:
                response.success = False
                response.message = f"Tool-rack failed to detect the unmounted tool with serial number '{tool_sn}'."
                goal_handle.abort()
                return response

            # move to tool_lifted pose with 'tool_mount_tcp'
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.cartesian_speed = 0.08
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_slide_in_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_slide_in pose."
                goal_handle.abort()
                return response

            # re-enable collisions between tool-mount and tool-rack after the mount operation
            self._planner_service.allow_collisions(model_id=tool_sn, allowed=False, tool_mount_link='tool_mount_tcp', slot_link='tool_rack_link')

            # reconfigure endtool as mounted
            self._endtool_mounted(tool_sn, False)

            response.success = True
            goal_handle.succeed()
            return response
    # endregion: callbacks

    # region: private methods
    def _get_full_launch_file_path(self, launch_file: str) -> str:
        """Resolve a bare endtool launch file name to its installed full path.

        Looks up the ``endtools`` package share directory and builds the path to
        the launch file inside its ``launch`` sub-directory, validating that the
        file exists on disk.

        :param launch_file: Launch file name (e.g. ``volumetric_dispenser_launch.py``).
        :returns: Absolute filesystem path to the launch file.
        :raises ActivationFailedException: If the ``endtools`` package cannot be
            found or the launch file does not exist.
        """
        try:
            share_directory = get_package_share_directory('endtools')
        except PackageNotFoundError as error:
            raise ActivationFailedException(
                f"Failed to locate 'endtools' package share directory: {error}"
            ) from error

        full_path = os.path.join(share_directory, 'launch', launch_file)
        if not os.path.isfile(full_path):
            raise ActivationFailedException(
                f"Launch file '{launch_file}' not found at expected path '{full_path}'."
            )
        return full_path

    def _wait_for_future(self, future, timeout: float) -> bool:
        """Wait for a future to complete without nested spinning.

        Polls ``future.done()`` until it resolves or the timeout elapses. The
        future is serviced by another thread of the node's
        :class:`~rclpy.executors.MultiThreadedExecutor`, so this can safely run
        on an executor thread. The timeout is measured against the node clock so
        that it honours ``use_sim_time``.

        :param future: The future to wait on.
        :param timeout: Maximum time to wait, in seconds.
        :returns: ``True`` if the future completed in time, ``False`` otherwise.
        """
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while not future.done():
            if self.get_clock().now() > deadline:
                return False
            self.get_clock().sleep_for(Duration(seconds=0.01))
        return True

    def _call_action(self, client: ActionClient, goal: ExecuteTrajectory.Goal, timeout: float=5.0) -> bool:
        """Send an action goal and wait for a successful result.

        :param client: Action client to call; ``None`` yields an instant ``False``.
        :param goal: The goal message to send.
        :param timeout: Per-step timeout (server availability, goal acceptance,
            and result) in seconds.
        :returns: ``True`` only if the goal was accepted and the action completed
            with a succeeded status and a successful result; ``False`` otherwise.
        """
        if client is None:
            self.get_logger().error('Cannot call action: action client is None.')
            return False

        if not client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Action server not available.')
            return False

        goal_future = client.send_goal_async(goal)
        if not self._wait_for_future(goal_future, timeout):
            self.get_logger().error('Timed out waiting for action goal to be accepted.')
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Action goal was rejected.')
            return False

        # Wait indefinitely for the result: this action drives a robot movement
        # whose duration is unknown, so there is no meaningful timeout to apply.
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            self.get_clock().sleep_for(Duration(seconds=0.01))

        result_wrapper = result_future.result()
        if result_wrapper is None or result_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Action did not complete with a succeeded status.')
            return False

        if not result_wrapper.result.success:
            self.get_logger().error(f'Action reported failure: {result_wrapper.result.error_message}')
            return False

        return True

    def _call_service(self, client: ServiceClient, request, timeout: float=5.0):
        """Call a service and wait for its response.

        :param client: Service client to call; ``None`` yields an instant ``False``.
        :param request: The request message to send.
        :param timeout: Timeout for service availability and response, in seconds.
        :returns: The service response on success, or ``False`` if the client is
            ``None``, the service is unavailable, or the call times out.
        """
        if client is None:
            self.get_logger().error('Cannot call service: service client is None.')
            return False

        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Service not available.')
            return False

        future = client.call_async(request)
        if not self._wait_for_future(future, timeout):
            self.get_logger().error('Timed out waiting for service response.')
            return False

        return future.result()

    def _get_frames_and_pose(self, tool_info: ToolInfoDto|ToolSlotDTO) -> tuple[SlotFramesDto, Pose]|None:
        frames_dto = SlotFramesDto(
            tool_slide_in_frame=tool_info.tool_slide_in_frame,
            tool_attached_frame=tool_info.tool_attached_frame,
            tool_lifted_frame=tool_info.tool_lifted_frame
        )

        # prepear pose rotated 180 degrees about the Z axis (no translation)
        pose = Pose()  
        pose.position.x = 0.0001
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 1.0
        pose.orientation.w = 0.0

        return frames_dto, pose

    def _endtool_mounted(self, tool_sn: str, mounted: bool) -> None:
        """Reconfigure a launched endtool node to reflect its mounted state.

        The endtool node reads its ``mounted`` parameter during ``on_configure``,
        so the state is updated by moving the node back to the unconfigured state,
        setting the ``mounted`` parameter, and configuring it again.

        :param tool_sn: Serial number identifying the launched endtool.
        :param mounted: ``True`` if the tool is now mounted on the tool-mount,
            ``False`` if it is on the rack.
        """
        endtool_launch = self._endtools.get(tool_sn)
        if endtool_launch is None:
            self.get_logger().error(
                f"Cannot reconfigure endtool '{tool_sn}': no launched endtool found."
            )
            return

        manager = endtool_launch.endtool_node_manager

        if manager.unconfigure_node() != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error(
                f"Failed to unconfigure endtool node for tool '{tool_sn}'."
            )
            return

        if not manager.set_parameter(
            'mounted',
            ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=mounted),
        ):
            self.get_logger().error(
                f"Failed to set 'mounted' parameter on endtool node for tool '{tool_sn}'."
            )
            return

        if manager.configure_node() != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error(
                f"Failed to configure endtool node for tool '{tool_sn}'."
            )
    # endregion: private methods


def main(args=None) -> None:
    """Entry point for the ``tool_mount`` executable.

    Initialises rclpy, creates a :class:`ToolMount` node, and spins
    it with a :class:`~rclpy.executors.MultiThreadedExecutor` (5 threads) to
    allow concurrent goal, feedback, and cancel callbacks.  Shuts down cleanly
    on exit or keyboard interrupt.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = ToolsManager()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        shutdown()
