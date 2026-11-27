# Copyright (c) 2026, Autofactory
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

from threading import RLock
from uuid import uuid4

from rclpy import init, ok, shutdown
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.parameter import ParameterValue, ParameterType
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Client as ServiceClient
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import SetBool

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath
from movement_controller.services.movement_session import MovementSession
from movement_controller.srv import CloseSession, InSession
from station_utils.autofactory_base_node import AutofactoryBaseNode
from station_utils.enums import LifecycleStateEnum
from station_utils.services.node_manager import NodeManager
from tools_manager.msg import Slots, ToolInfo
from tools_manager.action import MountTool, UnmountTool
from tools_manager.srv import GetToolNodeName

from tools_manager.exception.activation_failed_exception import ActivationFailedException
from tools_manager.exception.deactivation_failed_exception import DeactivationFailedException
from tools_manager.exception.cleanup_failed_exception import CleanupFailedException
from tools_manager.exception.endtool_reconfiguration_exception import EndtoolReconfigurationException
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.slot_frames_dto import SlotFramesDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.endtool_dto import EndtoolDto
from tools_manager.model.tools_manager_node_config import ToolsManagerNodeConfigDTO
from tools_manager.model.tools_manager_config import ToolSlotDTO
from tools_manager.services.world_client_adapter import ToolsWorldClientAdapter
from tools_manager.utils.inventory_validation import find_inventory_problems
from tools_manager.utils.node_parameters import (
    declare_simulation_parameters,
    declare_tools_parameters,
    read_simulation_inventory,
    read_tools_config,
)

from world_manager.exceptions import WorldManagerError

class ToolsManager(AutofactoryBaseNode):

    def __init__(self, node_name: str = 'tools_manager') -> None:
        """Initialise the ToolsManager lifecycle node.

        Declares all ROS 2 parameters with defaults and descriptions and
        creates the internal thread-safety primitives used to serialise
        concurrent goal and cancel callbacks.

        :param node_name: ROS 2 node name passed to :class:`AutofactoryBaseNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self._world_client_adapter: ToolsWorldClientAdapter | None = None
        self._tool_mount_manager: NodeManager | None = None
        self._tool_rack_manager: NodeManager | None = None
        # rack and mount nodes brought up by this manager, in the order they were configured
        self._sensing_managers: list[NodeManager] = []
        self._endtools: dict[str, EndtoolDto] = {}
        self._config: ToolsManagerNodeConfigDTO|None = None

        self._slots_info: SlotsDto|None = None
        self._slots_info_lock = RLock()

        # ``None`` with ``_tool_mounted_observed`` set means the mount reported itself empty;
        # ``None`` without it means no mount message has been received yet.
        self._tool_mounted_info: ToolInfoDto|None = None
        self._tool_mounted_observed = False
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
        declare_tools_parameters(self)
        declare_simulation_parameters(self)

        # subscriptions and publishers
        self._slots_subscription: Subscription|None = None
        self._tool_mounted_subscription: Subscription|None = None

        # service clients
        self._tool_mount_lock: ServiceClient|None = None
        self._in_session_client: ServiceClient|None = None
        self._close_session_client: ServiceClient|None = None

        # action servers and clients
        self._mount_tool_action: ActionServer|None = None
        self._unmount_tool_action: ActionServer|None = None

        # service servers
        self._get_tool_node_name_service: Service|None = None

        self._movement_controller_action_client: ActionClient|None = None
        self._movement_session: MovementSession|None = None

        # callback groups
        self._action_callback_group = ReentrantCallbackGroup()
        self._subscription_callback_group = ReentrantCallbackGroup()

    # region: properties
    @property
    def tool_rack_slots(self) -> SlotsDto|None:
        with self._slots_info_lock:
            return self._slots_info

    @tool_rack_slots.setter
    def tool_rack_slots(self, value: SlotsDto|None) -> None:
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

    @property
    def tool_mount_observed(self) -> bool:
        """Whether the tool mount has reported its state, including an empty mount."""
        with self._tool_mounted_info_lock:
            return self._tool_mounted_observed

    # endregion: properties

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            self._publish_transition_error('Base configure transition failed')
            return TransitionCallbackReturn.FAILURE

        try:
            simulated = self.get_parameter('simulated').get_parameter_value().bool_value
            self._config = ToolsManagerNodeConfigDTO(
                tools_manager_config=read_tools_config(self),
                simulation=read_simulation_inventory(self) if simulated else None,
                simulated=simulated,
                world_name=self.get_parameter('world_name').get_parameter_value().string_value,
                station_model_name=self.get_parameter('station_model_name').get_parameter_value().string_value,
                tool_mount_node_name=self.get_parameter('tool_mount_node_name').get_parameter_value().string_value,
                tool_rack_node_name=self.get_parameter('tool_rack_node_name').get_parameter_value().string_value,
                movement_controller_node_name=self.get_parameter('movement_controller_node_name').get_parameter_value().string_value,
            )
            self.get_logger().info(f'Tools manager configuration: {self._config}')
        except ValueError as e:
            self.get_logger().error(f'Invalid tools manager parameters: {e}')
            self._config = None
            self._publish_transition_error(f'Invalid tools manager parameters')
            return TransitionCallbackReturn.FAILURE

        try:
            self.get_logger().info('Creating world manager clients')
            self._world_client_adapter = ToolsWorldClientAdapter(
                node=self,
                callback_group=self._action_callback_group,
                simulated=self._config.simulated,
                station_model_name=self._config.station_model_name,
            )

            self.get_logger().info('Creating node state managers')
            self._tool_rack_manager = NodeManager(self, self._config.tool_rack_node_name)
            self._tool_mount_manager = NodeManager(self, self._config.tool_mount_node_name)
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            self._release_configuration()
            self._publish_transition_error(f'Unknown error while configuring')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to activate node: super().on_activate() returned FAILURE')
            self._publish_transition_error('Failed to activate node: super().on_activate() returned FAILURE')
            return TransitionCallbackReturn.FAILURE

        # sanity checks before activating the node
        if (
            self._world_client_adapter is None or
            self._config is None or
            self._tool_rack_manager is None or
            self._tool_mount_manager is None
        ):
            self.get_logger().error('Sanity check failed, something is un-initialized. Cannot activate node.')
            self._publish_transition_error('Sanity check failed, something is un-initialized. Cannot activate node.')
            return TransitionCallbackReturn.FAILURE

        try:
            self._activate()
        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            for failure in self._release_activation():
                self.get_logger().error(f'Rollback failure: {failure}')
            self._publish_transition_error(f'Activation failed: {e}')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to deactivate node: super().on_deactivate() returned FAILURE')
            raise DeactivationFailedException('Failed to deactivate node: super().on_deactivate() returned FAILURE')

        failures = self._release_activation()
        if failures:
            for failure in failures:
                self.get_logger().error(failure)
            raise DeactivationFailedException('; '.join(failures))

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to cleanup node: super().on_cleanup() returned FAILURE')
            raise CleanupFailedException('Failed to cleanup node: super().on_cleanup() returned FAILURE')

        self._release_configuration()

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
        with self._tool_mounted_info_lock:
            self._tool_mounted_info = tool_mounted if tool_mounted.tool_sn else None
            self._tool_mounted_observed = True

    def _cancel_tool_action_callback(self, goal_handle) -> bool:
        self.get_logger().info(f'Cancel request received for goal: {goal_handle}')
        return True  # TODO: implement cancel logic

    def _mount_tool_callback(self, goal_handle: ServerGoalHandle) -> MountTool.Result:
        """Execute the mount goal; all its movements run in one movement session."""
        return self._run_in_movement_session(goal_handle, MountTool.Result, self._mount_tool_impl)

    def _unmount_tool_callback(self, goal_handle: ServerGoalHandle) -> UnmountTool.Result:
        """Execute the unmount goal; all its movements run in one movement session."""
        return self._run_in_movement_session(goal_handle, UnmountTool.Result, self._unmount_tool_impl)

    def _run_in_movement_session(self, goal_handle: ServerGoalHandle, result_type, operation):
        """Run a mount/unmount operation so that all its movements form one atomic session.

        The tool action lock is taken first so a rejected concurrent request never
        touches the session of the running operation. The session itself is opened by
        the operation's first movement and always closed afterwards.
        """
        if self._tool_action_lock.acquire(blocking=False) is False:
            response = result_type()
            response.success = False
            response.message = "Another mount/unmount operation is in progress. Please try again later."
            goal_handle.abort()
            return response

        try:
            self._movement_session = MovementSession(
                in_session_call=lambda request: self._call_service(self._in_session_client, request) or None,
                close_session_call=lambda request: self._call_service(self._close_session_client, request) or None,
                now_s=lambda: self.get_clock().now().nanoseconds / 1e9,
            )
            try:
                return operation(goal_handle)
            finally:
                close_error = self._movement_session.close()
                if close_error:
                    self.get_logger().error(f'Failed to release movement session: {close_error}')
                self._movement_session = None
        finally:
            self._tool_action_lock.release()

    def _mount_tool_impl(self, goal_handle: ServerGoalHandle) -> MountTool.Result:
        """Mount the requested tool, returning the mounted one first when needed.

        Only called through :meth:`_run_in_movement_session`, which holds the tool
        action lock and the movement session for the whole operation, including
        the unmount that is part of a swap.
        """
        request: MountTool.Goal = goal_handle.request
        response = MountTool.Result()

        tool_rack_slots = self.tool_rack_slots
        tool_mounted = self.tool_mounted

        if tool_mounted is not None and tool_mounted.tool_sn == request.tool_sn:
            response.success = True
            response.message = f"Tool with serial number '{request.tool_sn}' is already mounted."
            goal_handle.succeed()
            return response

        if tool_rack_slots is None:
            response.success = False
            response.message = "Tool rack slots info is not available. Cannot perform mount operation."
            goal_handle.abort()
            return response

        # sanity check
        if (
            self._world_client_adapter is None or
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

        # wrong tool is mounted: swap by returning it to its slot first, within the same
        # lock and movement session; the unmount aborts this same goal on failure
        if tool_mounted is not None:
            self.get_logger().info(
                f"Tool '{tool_mounted.tool_sn}' is mounted, unmounting it before mounting '{tool_sn}'."
            )
            unmount_result = self._unmount_tool(
                goal_handle, tool_mounted.tool_sn, finalize=False,
            )
            if not unmount_result.success:
                response.success = False
                response.message = f"Failed to unmount tool '{tool_mounted.tool_sn}': {unmount_result.message}"
                return response

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
        if not self._world_client_adapter.allow_collisions(
            model_id=tool_sn,
            allowed=True,
            tool_mount_link='tool_mount_tcp',
            slot_link='tool_rack_link',
        ):
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
            self._world_client_adapter.attach_to_link(
                model_id=tool_sn,
                link_name='tool_mount_tcp',
                rotation=(0.0, 0.0, 1.0, 0.0),
            )
        except WorldManagerError as e:
            self.get_logger().error(f'Failed to attach tool to tool-mount: {e}')
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
        self._world_client_adapter.allow_collisions(
            model_id=tool_sn,
            allowed=False,
            tool_mount_link='tool_mount_tcp',
            slot_link='tool_rack_link',
        )

        # reconfigure endtool as mounted; the physical operation is already done at this point
        try:
            self._endtool_mounted(tool_sn, True)
        except EndtoolReconfigurationException as error:
            self.get_logger().error(f"Endtool reconfiguration after mounting '{tool_sn}' failed: {error}")
            response.success = False
            response.message = (
                f"Tool '{tool_sn}' was mounted, but its endtool could not be reconfigured: {error}"
            )
            goal_handle.abort()
            return response

        response.success = True
        goal_handle.succeed()
        return response

    def _unmount_tool_impl(self, goal_handle: ServerGoalHandle) -> UnmountTool.Result:
        """Return the requested tool to its rack slot only if it is mounted."""
        request: UnmountTool.Goal = goal_handle.request
        return self._unmount_tool(goal_handle, request.tool_sn)

    def _unmount_tool(
        self, goal_handle: ServerGoalHandle, tool_sn: str, finalize: bool = True,
    ) -> UnmountTool.Result:
        """Return the specified mounted tool to its rack slot.

        The caller must hold the tool action lock and the movement session (see
        :meth:`_run_in_movement_session`).

        :param goal_handle: Goal handle to abort on failure (and succeed when ``finalize``).
        :param tool_sn: Serial number that must match the currently mounted tool.
        :param finalize: ``False`` when called as a step of a larger goal (tool swap) so
            the goal is not marked succeeded here.
        """
        response = UnmountTool.Result()

        tool_mounted = self.tool_mounted
        slots = self.tool_rack_slots
        if tool_mounted is None:
            response.success = False
            response.message = "No tool is currently mounted."
            goal_handle.abort()
            return response
        if tool_sn != tool_mounted.tool_sn:
            response.success = False
            response.message = (
                f"Cannot unmount tool '{tool_sn}': currently mounted tool is "
                f"'{tool_mounted.tool_sn}'."
            )
            self.get_logger().error(response.message)
            goal_handle.abort()
            return response

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
            self._world_client_adapter is None or
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
        if not self._world_client_adapter.allow_collisions(
            model_id=tool_sn,
            allowed=True,
            tool_mount_link='tool_mount_tcp',
            slot_link='tool_rack_link',
        ):
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

        # wait for a bit to ensure that the tool is fully unmounted (otherwise it will
        # be detached while the tool-mount is moving and tool will be unmounted with incorrect pose
        self.get_clock().sleep_for(Duration(seconds=0.5))  

        try:
            # detach tool from mount in planning scene and attach to tool-rack slot
            self._world_client_adapter.attach_to_link(
                model_id=tool_sn,
                link_name=result[0].tool_attached_frame,
            )
        except WorldManagerError as e:
            self.get_logger().error(f'Failed to attach tool to tool-rack: {e}')
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
        self._world_client_adapter.allow_collisions(
            model_id=tool_sn,
            allowed=False,
            tool_mount_link='tool_mount_tcp',
            slot_link='tool_rack_link',
        )

        # reconfigure endtool as not mounted; the physical operation is already done at this point
        try:
            self._endtool_mounted(tool_sn, False)
        except EndtoolReconfigurationException as error:
            self.get_logger().error(f"Endtool reconfiguration after unmounting '{tool_sn}' failed: {error}")
            response.success = False
            response.message = (
                f"Tool '{tool_sn}' was returned to its slot, but its endtool could not be reconfigured: {error}"
            )
            goal_handle.abort()
            return response

        response.success = True
        if finalize:
            goal_handle.succeed()
        return response

    def _get_tool_node_name_callback(
        self, request: GetToolNodeName.Request, response: GetToolNodeName.Response
    ) -> GetToolNodeName.Response:
        response.success = False
        response.node_name = ''
        response.error_message = ''
        try:
            if self._config is None:
                raise ValueError('tools manager is not configured')
            slots = self.tool_rack_slots
            tool_mounted = self.tool_mounted
            on_rack = slots is not None and slots.get_tool_info(request.tool_sn) is not None
            is_mounted = tool_mounted is not None and tool_mounted.tool_sn == request.tool_sn
            if not (on_rack or is_mounted):
                raise ValueError(
                    f"tool '{request.tool_sn}' is neither on the rack nor mounted"
                )
            slot = self._config.tools_manager_config.get_tool_slot_data(request.tool_sn)
            if not slot.node_name:
                raise ValueError(f"tool serial number '{request.tool_sn}' has no endtool node name")
            response.node_name = slot.node_name
            response.success = True
        except ValueError as error:
            self.get_logger().error(f'get_tool_node_name failed: {error}')
            response.error_message = str(error)
        return response
    # endregion: callbacks

    # region: private methods
    def _activate(self) -> None:
        """Bring the manager into service; raises on the first failure.

        Partial progress is recorded in the manager's attributes so that
        :meth:`_release_activation` can undo exactly what was done.
        """
        assert self._config is not None
        assert self._world_client_adapter is not None
        assert self._tool_rack_manager is not None
        assert self._tool_mount_manager is not None

        # a previous attempt that could not be rolled back completely must be cleaned up first
        leftovers = self._teardown_endtools()
        if leftovers:
            raise ActivationFailedException(
                f"Endtools from a previous activation could not be cleaned up: {'; '.join(leftovers)}"
            )
        self._reset_observations()

        self._world_client_adapter.activate()

        # configure and activate tool rack and tool mount
        for manager in (self._tool_rack_manager, self._tool_mount_manager):
            result = manager.configure()
            if not result.success:
                raise ActivationFailedException(
                    f'Failed to configure node [{manager.node_name}]: {result.error_message}')
            self._sensing_managers.append(manager)
            result = manager.activate()
            if not result.success:
                raise ActivationFailedException(
                    f'Failed to activate node [{manager.node_name}]: {result.error_message}')

        # create subscription for tool_rack and tool_mount topics that provide
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

        rack_tools, mounted_tool = self._get_initial_inventory()
        problems = find_inventory_problems(self._config.tools_manager_config, rack_tools, mounted_tool)
        if problems:
            raise ActivationFailedException(f"Tool inventory is not valid: {'; '.join(problems)}")

        self._attach_endtools(rack_tools, mounted_tool)
        for endtool in self._endtools.values():
            self._configure_endtool(endtool)
            self._create_endtool_model(endtool)

        self.get_logger().info('Creating interfaces for tools manager')
        self._create_interfaces()

    def _get_initial_inventory(self) -> tuple[list[ToolInfoDto], ToolInfoDto | None]:
        """Obtain the tools on the rack and on the mount from simulation parameters or hardware."""
        assert self._config is not None

        if self._config.simulated:
            self.get_logger().info('Running in simulation mode, using parameters to get tools installed and mounted.')
            assert self._config.simulation is not None
            return list(self._config.simulation.rack.values()), self._config.simulation.mount

        self.get_logger().info(
            'Running in hardware mode, waiting for tool rack and tool mount to publish installed and mounted tools info.'
        )
        for _ in range(10):
            slots = self.tool_rack_slots
            if slots is not None and self.tool_mount_observed:
                return list(slots.tools_mounted), self.tool_mounted
            self.get_clock().sleep_for(Duration(seconds=1.0))
        raise ActivationFailedException('Failed to get tool rack slots and/or tool mounted info after 10 attempts.')

    def _attach_endtools(self, rack_tools: list[ToolInfoDto], mounted_tool: ToolInfoDto | None) -> None:
        """Associate every detected tool with its mapped endtool node without changing the node.

        :raises ActivationFailedException: If a node is unavailable, belongs to another tool
            or does not provide its model.
        """
        assert self._config is not None

        located = [(tool, False) for tool in sorted(rack_tools, key=lambda tool: tool.index)]
        if mounted_tool is not None:
            located.append((mounted_tool, True))

        for tool, mounted in located:
            home_slot = self._config.tools_manager_config.get_tool_slot_data(tool.tool_sn)
            assert home_slot.node_name is not None
            manager = NodeManager(self, home_slot.node_name)
            try:
                model_name = self._read_endtool_model(manager, tool.tool_sn)
            except Exception:
                manager.destroy()
                raise
            self._endtools[tool.tool_sn] = EndtoolDto(
                tool=tool,
                home_slot=home_slot,
                node_manager=manager,
                model_name=model_name,
                mounted=mounted,
            )

    def _read_endtool_model(self, manager: NodeManager, tool_sn: str) -> str:
        """Verify that an endtool node is ready to be taken over and read its model name.

        The node must be unconfigured, belong to ``tool_sn`` and provide its
        read-only ``model`` parameter.
        """
        node_name = manager.node_name
        state_result = manager.get_state()
        if not state_result.success:
            raise ActivationFailedException(
                f"Endtool node '{node_name}' of tool '{tool_sn}' is unavailable: its lifecycle state cannot be read "
                f'({state_result.error_message}). Make sure the endtool node is running.'
            )
        state = state_result.state
        if state != LifecycleStateEnum.UNCONFIGURED:
            raise ActivationFailedException(
                f"Endtool node '{node_name}' of tool '{tool_sn}' is '{state.value}', expected 'unconfigured'. "
                'Make sure its pattern executor is inactive and the endtool was cleaned up.'
            )

        serial = manager.get_parameter('tool_sn').value
        if serial is None or serial.type != ParameterType.PARAMETER_STRING or serial.string_value != tool_sn:
            actual = serial.string_value if serial is not None else None
            raise ActivationFailedException(
                f"Endtool node '{node_name}' reports tool serial number {actual!r}, expected '{tool_sn}'."
            )

        model = manager.get_parameter('model').value
        if model is None or model.type != ParameterType.PARAMETER_STRING or not model.string_value:
            raise ActivationFailedException(
                f"Cannot read the 'model' parameter of endtool node '{node_name}' for tool '{tool_sn}'."
            )
        return model.string_value

    def _configure_endtool(self, endtool: EndtoolDto) -> None:
        """Set the mounted state of an endtool and configure it, leaving it inactive."""
        manager = endtool.node_manager
        if not manager.set_parameter(
            'mounted',
            ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=endtool.mounted),
        ).success:
            raise ActivationFailedException(
                f"Failed to set 'mounted' parameter on endtool node '{manager.node_name}'."
            )
        if not manager.configure().success:
            raise ActivationFailedException(
                f"Failed to configure endtool node '{manager.node_name}' for tool '{endtool.tool.tool_sn}'."
            )
        endtool.configured = True

    def _create_endtool_model(self, endtool: EndtoolDto) -> None:
        """Create the planning scene and simulation models of an endtool where it currently sits."""
        assert self._world_client_adapter is not None

        if endtool.mounted:
            link_name = 'tool_mount_tcp'
            rotation = (0.0, 0.0, 1.0, 0.0)
        else:
            link_name = endtool.home_slot.tool_attached_frame
            rotation = None
        self._world_client_adapter.spawn_tool(
            endtool.tool,
            model_name=endtool.model_name,
            home_slot=endtool.home_slot,
            link_name=link_name,
            rotation=rotation,
        )
        endtool.world_model_created = True

    def _create_interfaces(self) -> None:
        """Create the clients, action servers and services that serve mount/unmount requests."""
        assert self._config is not None

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

        # movement session clients: reserve the movement controller for a whole mount/unmount
        self._in_session_client = self.create_client(
            srv_type=InSession,
            srv_name=f'{self._config.movement_controller_node_name}/in_session',
        )
        self._close_session_client = self.create_client(
            srv_type=CloseSession,
            srv_name=f'{self._config.movement_controller_node_name}/close_session',
        )

        # create tool managing action servers
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

        self._get_tool_node_name_service = self.create_service(
            srv_type=GetToolNodeName,
            srv_name='~/get_tool_node_name',
            callback=self._get_tool_node_name_callback,
            callback_group=self._action_callback_group,
        )

    def _release_activation(self) -> list[str]:
        """Undo everything :meth:`_activate` did, limited to what was actually done.

        Endtool and executor processes are owned by launch and keep running.

        :returns: Descriptions of the steps that failed; empty when everything was released.
        """
        self._release_interfaces()
        failures = self._teardown_endtools()

        for manager in reversed(self._sensing_managers):
            error = self._return_to_unconfigured(manager)
            if error is not None:
                failures.append(error)
        self._sensing_managers = []

        self._release_subscriptions()
        return failures

    def _teardown_endtools(self) -> list[str]:
        """Remove the models of the managed endtools and return their nodes to ``unconfigured``.

        An endtool whose teardown fails stays registered so the failure is
        reported again and cleanup is retried on the next activation.

        :returns: Descriptions of the steps that failed.
        """
        failures: list[str] = []
        for tool_sn, endtool in list(self._endtools.items()):
            if endtool.world_model_created and self._world_client_adapter is not None:
                try:
                    self._world_client_adapter.delete_tool(tool_sn)
                    endtool.world_model_created = False
                except WorldManagerError as error:
                    failures.append(f"Failed to remove the model of tool '{tool_sn}': {error}")

            if endtool.configured:
                error_message = self._return_to_unconfigured(endtool.node_manager)
                if error_message is None:
                    endtool.configured = False
                else:
                    failures.append(error_message)

            if not endtool.configured and not endtool.world_model_created:
                endtool.node_manager.destroy()
                del self._endtools[tool_sn]
        return failures

    def _return_to_unconfigured(self, manager: NodeManager) -> str | None:
        """Take a node back to ``unconfigured``, deactivating it first when it is active.

        :returns: A failure description, or ``None`` on success.
        """
        state_result = manager.get_state()
        if not state_result.success:
            return f"Cannot read the lifecycle state of node '{manager.node_name}': {state_result.error_message}"
        state = state_result.state
        if state == LifecycleStateEnum.ACTIVE and not manager.deactivate().success:
            return f"Failed to deactivate node '{manager.node_name}'"
        if state in (LifecycleStateEnum.ACTIVE, LifecycleStateEnum.INACTIVE) and not manager.cleanup().success:
            return f"Failed to unconfigure node '{manager.node_name}'"
        return None

    def _release_interfaces(self) -> None:
        """Destroy the clients, action servers and services created for mount/unmount requests."""
        if self._mount_tool_action is not None:
            self._mount_tool_action.destroy()
            self._mount_tool_action = None

        if self._unmount_tool_action is not None:
            self._unmount_tool_action.destroy()
            self._unmount_tool_action = None

        if self._get_tool_node_name_service is not None:
            self.destroy_service(self._get_tool_node_name_service)
            self._get_tool_node_name_service = None

        if self._movement_controller_action_client is not None:
            self._movement_controller_action_client.destroy()
            self._movement_controller_action_client = None

        for attribute in ('_in_session_client', '_close_session_client', '_tool_mount_lock'):
            client = getattr(self, attribute)
            if client is not None:
                self.destroy_client(client)
                setattr(self, attribute, None)

    def _release_subscriptions(self) -> None:
        """Destroy the rack and mount subscriptions."""
        if self._slots_subscription is not None:
            self.destroy_subscription(self._slots_subscription)
            self._slots_subscription = None

        if self._tool_mounted_subscription is not None:
            self.destroy_subscription(self._tool_mounted_subscription)
            self._tool_mounted_subscription = None

    def _release_configuration(self) -> None:
        """Release everything created by :meth:`on_configure`."""
        if self._world_client_adapter is not None:
            self._world_client_adapter.destroy()
            self._world_client_adapter = None
        for attribute in ('_tool_rack_manager', '_tool_mount_manager'):
            manager = getattr(self, attribute)
            if manager is not None:
                manager.destroy()
                setattr(self, attribute, None)
        self._config = None

    def _reset_observations(self) -> None:
        """Forget rack and mount observations made before the current activation."""
        self.tool_rack_slots = None
        with self._tool_mounted_info_lock:
            self._tool_mounted_info = None
            self._tool_mounted_observed = False

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
        """Send a movement goal within the active movement session and wait for a successful result.

        The session is opened by the first movement and extended when its timeout is
        close to expiry; its ID is set on the goal.

        :param client: Action client to call; ``None`` yields an instant ``False``.
        :param goal: The goal message to send; ``session_id`` is filled in here.
        :param timeout: Per-step timeout (server availability, goal acceptance,
            and result) in seconds.
        :returns: ``True`` only if the goal was accepted and the action completed
            with a succeeded status and a successful result; ``False`` otherwise.
        """
        if client is None:
            self.get_logger().error('Cannot call action: action client is None.')
            return False

        session = self._movement_session
        if session is None:
            self.get_logger().error('Cannot move: no movement session is active.')
            return False
        session_error = session.ensure_valid()
        if session_error:
            self.get_logger().error(f'Cannot move: {session_error}')
            return False
        goal.session_id = session.session_id

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
        """Reconfigure a managed endtool node to reflect its mounted state.

        The endtool node reads its ``mounted`` parameter during ``on_configure``,
        so the state is updated by moving the node back to the unconfigured state,
        setting the ``mounted`` parameter, and configuring it again. The endtool
        (or its pattern executor) must be inactive for this to succeed.

        :param tool_sn: Serial number identifying the managed endtool.
        :param mounted: ``True`` if the tool is now mounted on the tool-mount,
            ``False`` if it is on the rack.
        :raises EndtoolReconfigurationException: If any reconfiguration step fails.
        """
        endtool = self._endtools.get(tool_sn)
        if endtool is None:
            raise EndtoolReconfigurationException(f"no endtool is managed for tool '{tool_sn}'")

        manager = endtool.node_manager

        if not manager.cleanup().success:
            raise EndtoolReconfigurationException(
                f"failed to unconfigure endtool node '{manager.node_name}' of tool '{tool_sn}'; "
                'make sure its pattern executor is inactive'
            )
        endtool.configured = False

        if not manager.set_parameter(
            'mounted',
            ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=mounted),
        ).success:
            raise EndtoolReconfigurationException(
                f"failed to set 'mounted' parameter on endtool node '{manager.node_name}' of tool '{tool_sn}'; "
                'the node is left unconfigured'
            )

        if not manager.configure().success:
            raise EndtoolReconfigurationException(
                f"failed to configure endtool node '{manager.node_name}' of tool '{tool_sn}'; "
                'the node is left unconfigured'
            )
        endtool.configured = True
        endtool.mounted = mounted
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ok():
            shutdown()
