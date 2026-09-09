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

from threading import RLock
from uuid import uuid4

from rclpy import init, shutdown
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import ParameterValue
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.service import Service
from rclpy.client import Client as ServiceClient

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath
from tools_manager.srv import MountTool, UnmountTool, FillSlot, FreeSlot

from tools_manager.utils.config_reader import read_tool_rack_config_file
from tools_manager.tools_manager.model.rack_config import RackConfigDTO
from tools_manager.tools_manager.model.slots_dto import SlotsDto
from tools_manager.tools_manager.model.tool_mount_node_config_dto import ToolMountNodeConfigDTO
from tools_manager.tools_manager.model.slot_frames_dto import SlotFramesDto
from tools_manager.tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.interface.tool_mount_controller import ToolMountController
from tools_manager.tools_manager.services.simulated_tool_mount_controller import SimulatedToolMountController
from tools_manager.tools_manager.services.hardware_tool_mount_controller import HardwareToolMountController
from tools_manager.tools_manager.services.gazebo_world_manager import GazeboWorldManager
from tools_manager.tools_manager.services.moveit2_world_manager import Moveit2WorldManager
from tools_manager.tools_manager.services.node_state_manager import NodeStateManager

class ToolMount(LifecycleNode):

    def __init__(self, node_name: str = 'tool_mount') -> None:
        """Initialise the ToolMount lifecycle node.

        Declares all ROS 2 parameters with defaults and descriptions and
        creates the internal thread-safety primitives used to serialise
        concurrent goal and cancel callbacks.

        :param node_name: ROS 2 node name passed to :class:`rclpy.lifecycle.LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self._gazebo_service: GazeboWorldManager|None = None
        self._planner_service: Moveit2WorldManager|None = None
        self._tool_mount_controller: ToolMountController|None = None
        self._tool_rack_manager: NodeStateManager|None = None
        self._endtools_managers: dict[str, NodeStateManager] = {}
        self._config: ToolMountNodeConfigDTO|None = None

        self._slots_info: SlotsDto|None = None
        self._slots_info_lock = RLock()

        self._service_lock = RLock()

        # region: parameters
        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(description='Whether the node is running in simulation mode'),
        )
        self.declare_parameter(
            'parent_frame_id',
            "tool0",
            ParameterDescriptor(description='Parent frame ID for the tool mount'),
        )
        self.declare_parameter(
            'config_file',
            "tool_rack_config.yaml",
            ParameterDescriptor(description='Path to the tool rack configuration file'),
        )
        self.declare_parameter(
            'tool_rack_node_name',
            "tool_rack",
            ParameterDescriptor(description='Tool rack node name to communicate with'),
        )
        self.declare_parameter(
            'endtool_node_names',
            [],
            ParameterDescriptor(description='Endtool node names to communicate with'),
        )
        self.declare_parameter(
            'movement_controller_node_name',
            "movement_controller",
            ParameterDescriptor(description='Movement controller node name to communicate with'),
        )

        # subscriptions and publishers
        self._tool_rack_slots_subscription: Subscription|None = None
        self._tool_mounted_publisher: Publisher|None = None

        # services
        self._mount_tool_service: Service|None = None
        self._unmount_tool_service: Service|None = None

        # action clients
        self._movement_controller_action_client: ActionClient|None = None

    # region: properties
    @property
    def tool_rack_slots_state(self) -> SlotsDto:
        with self._slots_info_lock:
            return self._slots_info

    @tool_rack_slots_state.setter
    def tool_rack_slots_state(self, value: SlotsDto) -> None:
        with self._slots_info_lock:
            self._slots_info = value

    # endregion: properties

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        try:
            rack_config = read_tool_rack_config_file(self.get_parameter('config_file').get_parameter_value().string_value)
            self._config = ToolMountNodeConfigDTO(
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                parent_frame_id=self.get_parameter('parent_frame_id').get_parameter_value().string_value,
                rack_config=rack_config,
                tool_rack_node_name=self.get_parameter('tool_rack_node_name').get_parameter_value().string_value,
                endtool_node_names=list(self.get_parameter('endtool_node_names').get_parameter_value().string_array_value),
            )

            self._gazebo_service = GazeboWorldManager(
                node=self,
                world_name=self.get_parameter("world_name").get_parameter_value().string_value,
                station_model_name=self.get_parameter("station_name").get_parameter_value().string_value,  # TODO: this is the Gazebo model name given when inputting SDF to Gazebo
                rack_config=rack_config,
                tool_mount_link="tool_mount_tcp", #TODO: hardcoded now, but change later
            )
            self._planner_service = Moveit2WorldManager(
                node=self,
                rack_config=rack_config,
            )
            if self._config.simulated:
                self._tool_mount_controller = SimulatedToolMountController(self, self._config)
            else:
                self._tool_mount_controller = HardwareToolMountController(self, self._config)
            self._tool_rack_manager = NodeStateManager(self, self.get_parameter('tool_rack_node_name').get_parameter_value().string_value)
            for name in self.get_parameter('endtool_node_names').get_parameter_value().string_array_value:
                self._endtools_managers[name] = NodeStateManager(self, name)

            self._tool_rack_slots_subscription = self.create_subscription(
                SlotsDto,
                f'{self.get_parameter("tool_rack_node_name").get_parameter_value().string_value}/slots',
                self._slots_info_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
        except ValidationError as e:
            self.get_logger().error(f'Failed to read tool rack config file: {e}')
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            self._gazebo_service = None
            self._planner_service = None
            self._tool_rack_manager = None
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        # sanity checks before activating the node
        if (
            self._tool_mount_controller is None or
            self._gazebo_service is None or
            self._planner_service is None or
            self._config is None or
            self._tool_rack_manager is None
        ):
            self.get_logger().error('Sanity check failed, something is un-initialized. Cannot activate node.')
            return TransitionCallbackReturn.FAILURE

        # configure and activate tool rack
        if self._tool_rack_manager.configure_node() != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to configure tool rack')
            return TransitionCallbackReturn.FAILURE
        if self._tool_rack_manager.activate_node() != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to activate tool rack')
            return TransitionCallbackReturn.FAILURE

        # establish communication with tool-mount hardware
        try:
            self._tool_mount_controller.setup()
        except ToolMountControllerError as e:
            self.get_logger().error(f'Failed to setup tool mount controller: {e}')
            return TransitionCallbackReturn.FAILURE

        # create tool mounted publisher # TODO: destory this publisher on deactivate
        self._tool_mounted_publisher = self.create_publisher(
            String,
            'tool_mounted',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # configure endtools (tool-mount never activates them!)
        for tool_name, manager in self._endtools_managers.items():
            mounted_tool = self._tool_mount_controller.get_mounted_tool_info()
            tool_sn = manager.get_parameter('tool_sn').string_value
            if mounted_tool and tool_sn == mounted_tool.tool_sn:
                mounted_parameter_value = ParameterValue(type=ParameterValue.BOOL, bool_value=True)
            else:
                mounted_parameter_value = ParameterValue(type=ParameterValue.BOOL, bool_value=False)

            if not manager.set_parameter('mounted', mounted_parameter_value):
                self.get_logger().error(f'Failed to set parameter "mounted" for endtool: {tool_name}')
                return TransitionCallbackReturn.FAILURE
            
            if manager.configure_node() != TransitionCallbackReturn.SUCCESS:
                self.get_logger().error(f'Failed to configure endtool: {tool_name}')
                return TransitionCallbackReturn.FAILURE
            # TODO: think about if we should deactivate and unconfigure tool-rack and endtools if fail

            if mounted_tool:
                self.get_logger().info(f'Endtool {mounted_tool.tool_sn} is mounted, attaching to tool-mount')
                string_msg = String(data=mounted_tool.tool_sn)
                self._tool_mounted_publisher.publish(string_msg)
                self._planner_service.attach_to_tool_mount(mounted_tool.tool_sn)
            else:
                self.get_logger().info(f'Endtool {tool_name} is not mounted, attaching to tool-rack')
                string_msg = String(data='')
                self._tool_mounted_publisher.publish(string_msg)
                self._planner_service.attach_to_tool_rack(tool_sn)
            
        if self._config.simulated:
            # TODO: ensure that these services get destoroyed on deactivate
            self._fill_slot_service = self.create_client(
                FillSlot,
                f'{self.get_parameter("tool_rack_node_name").get_parameter_value().string_value}/fill_slot'
            )

            self._free_slot_service = self.create_client(
                FreeSlot,
                f'{self.get_parameter("tool_rack_node_name").get_parameter_value().string_value}/free_slot'
            )

        # create movement controller action client # TODO: destroy this action client on deactivate
        self._movement_controller_action_client = ActionClient(
            self,
            ExecuteTrajectory,
            f'{self.get_parameter("movement_controller_node_name").get_parameter_value().string_value}/execute_trajectory',
        )

        # create services for mounting and unmounting tools #TODO: destroy those services on deactivate
        self._mount_tool_service = self.create_service(
            MountTool,
            'mount_tool',
            self._mount_tool_callback,
        )
        self._unmount_tool_service = self.create_service(
            UnmountTool,
            'unmount_tool',
            self._unmount_tool_callback,
        )

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)

    # endregion: lifecycle callbacks

    # region: callbacks
    def _slots_info_callback(self, msg: SlotsDto) -> None:
        self.get_logger().info(f'Received slots info update: {msg}')
        self.tool_rack_slots_state = msg

    def _mount_tool_callback(self, request: MountTool.Request, response: MountTool.Response) -> MountTool.Response:
        if self._service_lock.acquire(blocking=False) is False:
            response.success = False
            response.message = "Another mount/unmount operation is in progress. Please try again later."
            return response
        
        with self._service_lock:
            self._service_lock.release()  # release the lock as we already hold it
            if self._is_mounted:
                response.success = False
                response.message = "A tool is already mounted. Please unmount it first."
                return response

            # sanity check
            if (
                self._planner_service is None or
                self._gazebo_service is None or
                self._tool_mount_controller is None or
                self._movement_controller_action_client is None or
                self._config is None or
                self._tool_mounted_publisher is None
            ):
                response.success = False
                response.message = "Sanity check failed (something un-initialized). Cannot perform mount operation."
                return response
            elif (
                self._config.simulated and
                self._fill_slot_service is None or
                self._free_slot_service is None
            ):
                response.success = False
                response.message = "Sanity check failed (simulation services uninitialized). Cannot perform mount operation."
                return response

            # TODO: consider how failure should be handled - do we roll back or unconfigure or error?

            tool_sn = request.tool_sn
            tool_info = self.tool_rack_slots_state.get_tool_info(tool_sn)
            if tool_info is None:
                response.success = False
                response.message = f"Tool with serial number '{tool_sn}' is not on the rack."
                return response

            # get the parent frames for moving with 'tool_mount_tcp' frame/link; all frames are
            # defined in such a way that movement pose required is all zeros - this means that
            # 'tool_mount_tcp' frame must align with the target frame and then we are in correct pose
            result = self._get_frames_and_unity_pose(tool_info)
            if result is None:
                response.success = False
                response.message = f"Tool with serial number '{tool_sn}' is not on the rack."
                return response
            frames, unity_pose = result

            # move to tool_slide in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_PTP
            # TODO: define default speed parameters (both for PTP and LIN) for 'movement_controller' node - if provided values are negative then use default
            path.tool_frame = 'tool_mount_tcp'  # NOTE: this is hardcoded frame and matches the link defined in URDF; DO NOT CHANGE IT UNLESS CHANGING IN URDF ALSO!
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_slide_in_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_slide_in pose."

            # allow collisions between tool-mount, endtool and tool-rack for the duration of the mount operation
            if not self._planner_service.allow_collisions(tool_sn, True):   
                response.success = False
                response.message = "Failed to allow collisions between tool-mount, endtool and tool-rack."
                return response

            # operate tool-mount quick release to mount the tool
            if self._tool_mount_controller.lock_closed(False) is False:
                response.success = False
                response.message = "Failed to open tool-mount quick release."
                return response

            # move to tool_attached in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_attached_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_attached pose."

            # detach tool from rack in planning scene and attach to tool-mount
            self._planner_service.attach_to_tool_mount(tool_sn)  # TODO: ensure we use correct object ID here

            # if simulating then first attach the tool to the tool-mount in Gazebo and then detach it from the tool-rack
            if self._config.simulated:
                # attach first, otherwise tool will start fidgeting
                if not self._gazebo_service.attach_to_tool_mount(tool_sn):  # NOTE: tool serial number is model name/ID in Gazebo
                    response.success = False
                    response.message = "Failed to attach tool to tool-mount in Gazebo."

            # operate tool-mount quick release to mount the tool
            if self._tool_mount_controller.lock_closed(True) is False:
                response.success = False
                response.message = "Failed to close tool-mount quick release."
                return response

            # move to tool_lifted pose with 'tool_mount_tcp'
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_lifted_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            # re-enable collisions between tool-mount and tool-rack after the mount operation
            self._planner_service.allow_collisions(tool_sn, False)

            # reconfigure endtool as mounted
            self._endtool_mounted(tool_sn, True)

            # inform tool rack about tool being unmounted
            if self._config.simulated:
                if not self._call_service(self._free_slot_service, FreeSlot.Request(slot_id=tool_info.slot_id)):
                    response.success = False
                    response.message = "Failed to free slot in tool rack."

            # publish that tool-mount has tool mounted
            self._is_mounted = True
            string_msg = String(data=tool_sn)
            self._tool_mounted_publisher.publish(string_msg)

            response.success = True
            return response

    def _unmount_tool_callback(self, request: UnmountTool.Request, response: UnmountTool.Response) -> UnmountTool.Response:
        if self._service_lock.acquire(blocking=False) is False:
            response.success = False
            response.message = "Another mount/unmount operation is in progress. Please try again later."
            return response
        
        with self._service_lock:
            self._service_lock.release()  # release the lock as we already hold it
            if self._is_mounted is False:
                response.success = False
                response.message = "No tool is currently mounted."
                return response

            # sanity check
            if (
                self._planner_service is None or
                self._gazebo_service is None or
                self._tool_mount_controller is None or
                self._movement_controller_action_client is None or
                self._config is None or
                self._tool_mounted_publisher is None
            ):
                response.success = False
                response.message = "Sanity check failed (something un-initialized). Cannot perform unmount operation."
                return response
            elif (
                self._config.simulated and
                self._fill_slot_service is None or
                self._free_slot_service is None
            ):
                response.success = False
                response.message = "Sanity check failed (simulation services uninitialized). Cannot perform unmount operation."
                return response

            tool_info = self._tool_mount_controller.get_mounted_tool_info()
            if tool_info is None:
                response.success = False
                response.message = f"No tool is currently mounted on the tool-mount."
                return response
            else:
                tool_sn = tool_info.tool_sn

            # TODO: consider how failure should be handled - do we roll back or unconfigure or error?

            # get the parent frames for moving with 'tool_mount_tcp' frame/link; all frames are
            # defined in such a way that movement pose required is all zeros - this means that
            # 'tool_mount_tcp' frame must align with the target frame and then we are in correct pose
            result = self._get_frames_and_unity_pose(tool_info)
            if result is None:
                response.success = False
                response.message = f"Tool with serial number '{tool_sn}' is not on the rack."
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

            # allow collisions between tool-mount, endtool and tool-rack for the duration of the mount operation
            if not self._planner_service.allow_collisions(tool_sn, True):   
                response.success = False
                response.message = "Failed to allow collisions between tool-mount, endtool and tool-rack."
                return response

            # move to tool_attached in pose with 'tool_mount_tcp' 
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_attached_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            if not self._call_action(self._movement_controller_action_client, ExecuteTrajectory.Goal(paths=[path])):
                response.success = False
                response.message = "Failed to move to tool_attached pose."

            # detach tool from rack in planning scene and attach to tool-mount
            self._planner_service.attach_to_tool_rack(tool_sn)  # TODO: ensure we use correct object ID here

            # if simulating then first attach the tool to the tool-rack in Gazebo and then detach it from the tool-mount
            if self._config.simulated:
                # attach first, otherwise tool will start fidgeting
                if not self._gazebo_service.attach_to_tool_rack(tool_sn):
                    response.success = False
                    response.message = "Failed to attach tool to tool-rack in Gazebo."

            # operate tool-mount quick release to mount the tool
            if self._tool_mount_controller.lock_closed(False) is False:
                response.success = False
                response.message = "Failed to open tool-mount quick release."
                return response

            # move to tool_lifted pose with 'tool_mount_tcp'
            path = TrajectoryPath()
            path.path_id = str(uuid4())
            path.motion_type = TrajectoryPath.MOTION_TYPE_LIN
            path.tool_frame = 'tool_mount_tcp'
            path.target_pose = PoseStamped()
            path.target_pose.header.frame_id = frames.tool_slide_in_frame
            path.target_pose.header.stamp = self.get_clock().now().to_msg()
            path.target_pose.pose = unity_pose

            # re-enable collisions between tool-mount and tool-rack after the mount operation
            self._planner_service.allow_collisions(tool_sn, False)

            # reconfigure endtool as mounted
            self._endtool_mounted(tool_sn, False)

            # inform tool rack about tool being unmounted
            if self._config.simulated:
                if not self._call_service(self._free_slot_service, FillSlot.Request(slot_id=tool_info.slot_id)):
                    response.success = False
                    response.message = "Failed to fill slot in tool rack."

            # publish that tool-mount has tool mounted
            self._is_mounted = False  # TODO: in sim we set this, but in hardware mode it should come from controller - need to unify this logic
            string_msg = String(data=tool_sn)
            self._tool_mounted_publisher.publish(string_msg)

            response.success = True
            return response
    # endregion: callbacks

    # region: private methods
    def _call_action(self, client: ActionClient, goal: ExecuteTrajectory.Goal, timeout: float=5.0) -> bool:
        raise NotImplementedError()
        # TODO: implement full action call - it should return 'True' if successfully completed or
        # 'False' if any failure (goal not accepted, aborted etc); if action client is None then it is instant 'False'

    def _call_service(self, client: ServiceClient, request, timeout: float=5.0) -> bool:
        raise NotImplementedError()
        # TODO: implement full service call - it should return the response if successfully completed or
        # 'False' if any failure (service not available, timeout etc); if service client is None then it is instant 'False'

    def _get_frames_and_unity_pose(self, tool_info: ToolInfoDto) -> tuple[SlotFramesDto, Pose]|None:
        frames_dto = SlotFramesDto(
            tool_slide_in_frame= tool_info.tool_slide_in_frame,
            tool_attached_frame= tool_info.tool_attached_frame,
            tool_lifted_frame = tool_info.tool_lifted_frame
        )

        # prepear unity pose - pose that does zero movement
        unity_pose = Pose()  
        unity_pose.position.x = 0.0
        unity_pose.position.y = 0.0
        unity_pose.position.z = 0.0
        unity_pose.orientation.x = 0.0
        unity_pose.orientation.y = 0.0
        unity_pose.orientation.z = 0.0
        unity_pose.orientation.w = 1.0

        return frames_dto, unity_pose

    def _endtool_mounted(self, tool_sn: str, mounted: bool) -> None:
        raise NotImplementedError()
        # TODO: implement method that unconfigures endtool node, sets 'mounted' parameter and then configures
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
    node = ToolMount()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        shutdown()
