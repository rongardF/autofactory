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
"""Common lifecycle node of the station workplace nodes."""

import threading
from typing import Any

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from std_msgs.msg import Bool

from station_workplace.enums.constants import ATTACH_LINK
from station_workplace.enums.process_type_enum import ProcessTypeEnum
from station_workplace.interface.product_data_reader import ProductDataReader
from station_workplace.interface.simulation_hooks import SimulationHooks
from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO, SafetyObjectConfigDTO
from station_workplace.models.simulation_config_dto import SimulationConfigDTO
from station_workplace.services.gazebo_world_manager import GazeboWorldManager
from station_workplace.services.moveit2_planning_scene_manager import Moveit2PlanningSceneManager
from station_workplace.services.process_runner import ProcessRunner
from station_workplace.services.product_origin_tf_publisher import ProductOriginTfPublisher
from station_workplace.services.simulated_product_spawner import SimulatedProductSpawner
from station_workplace.services.wait_routing_action_server import WaitRoutingActionServer
from station_workplace.services.workplace_process_service import WorkplaceProcessService
from station_workplace.services.workplace_state_publisher import WorkplaceStatePublisher
from station_workplace.services.workplace_state_tracker import WorkplaceStateTracker


class WorkplaceNodeBase(LifecycleNode):
    """Lifecycle node reacting to ``product_in_place`` with injection/ejection.

    Subclasses only provide the node-specific parameters, configuration DTO and
    :class:`ProductDataReader`.

    Interfaces (created only while active):

    * ``~/workplace`` (``station_workplace/WorkplaceState``, latched);
    * ``~/wait_routing`` (``station_workplace/WaitRouting`` action);
    * TF ``estimated_product_origin`` (parent ``carrier``) on ``/tf_static``;
    * ``~/spawn_product`` and ``~/remove_product`` services, only if ``simulated``.
    """

    _PRODUCT_IN_PLACE_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, node_name: str) -> None:
        """Declare the parameters; resources are created by lifecycle transitions."""
        super().__init__(node_name)
        defaults = CommonNodeConfigDTO()
        safety = SafetyObjectConfigDTO()
        self._declare('product_carrier_node_name', defaults.product_carrier_node_name,
                      'Product carrier node name.')
        self._declare('production_manager_node_name', defaults.production_manager_node_name,
                      'Production manager node name.')
        self._declare('object_model_repository_node_name',
                      defaults.object_model_repository_node_name,
                      'Object model storage node name.')
        self._declare('collision_allowed_link', defaults.collision_allowed_link,
                      'Robot link the carrier model and the safety object may collide with.')
        self._declare('safety_object.x', safety.x, 'Safety box size along X [m] (> 0).')
        self._declare('safety_object.y', safety.y, 'Safety box size along Y [m] (> 0).')
        self._declare('safety_object.z', safety.z, 'Safety box size along Z [m] (> 0).')
        self._declare('service_call_timeout_sec', defaults.service_call_timeout_sec,
                      'Timeout for service availability and response [s] (> 0).')
        self._declare('simulated', defaults.simulated,
                      'Provide the spawn_product/remove_product simulation services.')
        for name, field in SimulationConfigDTO.model_fields.items():
            self._declare(f'simulation.{name}', field.default, field.description or name)
        self._declare_specific_parameters()

        self._trigger_group = MutuallyExclusiveCallbackGroup()
        self._io_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        self._config: CommonNodeConfigDTO | None = None
        self._scene: Moveit2PlanningSceneManager | None = None
        self._reader: ProductDataReader | None = None
        self._process: WorkplaceProcessService | None = None
        self._runner: ProcessRunner | None = None
        self._routing: WaitRoutingActionServer | None = None
        self._spawner: SimulatedProductSpawner | None = None
        # Created once so the record of spawned models survives activation cycles.
        self._world = GazeboWorldManager(self)
        # The publishers and the tracker keep their state (last published value,
        # TF published flag, settled state) across activations and are therefore created once.
        self._state_tracker = WorkplaceStateTracker()
        self._state_publisher = WorkplaceStatePublisher(on_settled=self._state_tracker.settle)
        self._tf_publisher = ProductOriginTfPublisher()
        self._trigger_sub: Subscription | None = None

        self._is_active = False
        self._first_after_activation = True
        self._last_handled: bool | None = None

        self._fatal_lock = threading.Lock()
        self._fatal_pending = False
        # Wakes the executor from any thread to run the deferred self-transition.
        self._fatal_guard = self.create_guard_condition(self._on_fatal_guard)

    # region subclass hooks
    def _declare_specific_parameters(self) -> None:
        """Declare the node-specific parameters."""
        raise NotImplementedError

    def _build_config(self, common: dict[str, Any]) -> CommonNodeConfigDTO:
        """Build the node-specific config DTO from the common parameter values.

        :raises ValidationError: If a parameter is invalid.
        """
        raise NotImplementedError

    def _create_reader(self, config: CommonNodeConfigDTO) -> ProductDataReader:
        """Create the node-specific product data reader."""
        raise NotImplementedError

    def _create_simulation_hooks(self, config: CommonNodeConfigDTO) -> SimulationHooks:
        """Create the node-specific hooks of the simulated product spawning."""
        raise NotImplementedError
    # endregion subclass hooks

    # region parameters
    def _declare(self, name: str, default: Any, description: str) -> None:
        self.declare_parameter(name, default, ParameterDescriptor(description=description))

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _common_parameters(self) -> dict[str, Any]:
        return {
            'product_carrier_node_name': self._param('product_carrier_node_name'),
            'production_manager_node_name': self._param('production_manager_node_name'),
            'object_model_repository_node_name': self._param('object_model_repository_node_name'),
            'collision_allowed_link': self._param('collision_allowed_link'),
            'safety_object': {
                'x': self._param('safety_object.x'),
                'y': self._param('safety_object.y'),
                'z': self._param('safety_object.z'),
            },
            'service_call_timeout_sec': self._param('service_call_timeout_sec'),
            'simulated': self._param('simulated'),
            'simulation': {
                name: self._param(f'simulation.{name}')
                for name in SimulationConfigDTO.model_fields
            },
        }
    # endregion parameters

    # region lifecycle transitions
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate the parameters and construct the internal services."""
        try:
            config = self._build_config(self._common_parameters())
        except ValidationError as exc:
            for error in exc.errors():
                location = '.'.join(str(part) for part in error['loc'])
                self.get_logger().error(f"Invalid parameter '{location}': {error['msg']}")
            return TransitionCallbackReturn.FAILURE

        try:
            runner = ProcessRunner(self.get_logger())
            scene = Moveit2PlanningSceneManager(
                self,
                attach_link=ATTACH_LINK,
                collision_allowed_link=config.collision_allowed_link,
                safety_size=config.safety_object,
                timeout_sec=config.service_call_timeout_sec,
                callback_group=self._io_group,
            )
            reader = self._create_reader(config)
            process = WorkplaceProcessService(
                self, config, scene, self._state_publisher, self._tf_publisher,
                reader, runner, self._io_group, self._on_system_failure,
            )
            routing = WaitRoutingActionServer(
                self, config, self._state_tracker, self._action_group, self._io_group)
            spawner = None
            if config.simulated:
                spawner = SimulatedProductSpawner(
                    self, config, self._world, self._create_simulation_hooks(config),
                    self._io_group)
        except Exception as exc:  # noqa: BLE001 - convert to transition failure
            self.get_logger().error(f'Failed to construct the internal services: {exc}')
            return TransitionCallbackReturn.FAILURE

        self._config = config
        self._runner = runner
        self._scene = scene
        self._reader = reader
        self._process = process
        self._routing = routing
        self._spawner = spawner
        self.get_logger().info(f'Configured: {config}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create all interfaces and enable injection/ejection handling."""
        with self._fatal_lock:
            self._fatal_pending = False
        if (
            self._config is None or self._scene is None or self._reader is None
            or self._process is None or self._runner is None or self._routing is None
        ):
            self.get_logger().error('Activation failed: node is not configured')
            return TransitionCallbackReturn.FAILURE
        try:
            self._scene.activate()
            self._reader.activate(self, self._io_group)
            self._process.activate()
            self._state_publisher.activate(self)
            self._tf_publisher.activate(self)
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError(f'base activation returned {result}')
            self._first_after_activation = True
            self._is_active = True
            self._runner.enable()
            self._routing.activate()
            if self._spawner is not None:
                self._spawner.activate()
            # Created last: the latched value may be delivered immediately.
            self._trigger_sub = self.create_subscription(
                Bool, self._config.product_in_place_topic, self._on_product_in_place,
                self._PRODUCT_IN_PLACE_QOS, callback_group=self._trigger_group)
        except Exception as exc:  # noqa: BLE001 - tear down and fail the transition
            self.get_logger().error(f'Activation failed: {exc}')
            self._is_active = False
            self._runner.disable()
            self._destroy_interfaces()
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info(f'Activated (listening on {self._config.product_in_place_topic})')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Abandon running work, ignore later triggers and destroy the interfaces."""
        self._is_active = False
        if self._runner is not None:
            self._runner.disable()
        super().on_deactivate(state)
        self._destroy_interfaces()
        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the internal services and the configuration."""
        self._drop_services()
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release all resources from any state; planning-scene objects stay."""
        self._release(state)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Release all resources and return to unconfigured."""
        self.get_logger().error('Error transition: releasing all resources')
        self._release(state)
        return TransitionCallbackReturn.SUCCESS
    # endregion lifecycle transitions

    # region resource handling
    def _destroy_interfaces(self) -> None:
        """Abort a running wait_routing goal; destroy the action server, subscription, clients and publishers."""
        if self._trigger_sub is not None:
            self.destroy_subscription(self._trigger_sub)
            self._trigger_sub = None
        components = (self._spawner, self._routing, self._reader, self._process, self._scene)
        for component in components:
            if component is not None:
                try:
                    component.deactivate()
                except Exception as exc:  # noqa: BLE001 - keep releasing
                    self.get_logger().error(f'Failed to deactivate {type(component).__name__}: {exc}')
        self._state_publisher.deactivate()
        self._tf_publisher.deactivate()

    def _drop_services(self) -> None:
        self._spawner = None
        self._routing = None
        self._state_tracker.reset()
        self._process = None
        self._reader = None
        self._scene = None
        self._runner = None
        self._config = None
        self._state_publisher.reset()
        self._last_handled = None

    def _release(self, state: State) -> None:
        was_active = self._is_active
        self._is_active = False
        if self._runner is not None:
            self._runner.disable()
        if was_active:
            super().on_deactivate(state)
        self._destroy_interfaces()
        self._drop_services()
    # endregion resource handling

    # region trigger
    def _on_product_in_place(self, msg: Bool) -> None:
        """Queue injection (True) or ejection (False); never blocks."""
        runner, process, reader = self._runner, self._process, self._reader
        if not self._is_active or runner is None or process is None or reader is None:
            return
        value = bool(msg.data)
        if not self._first_after_activation and value == self._last_handled:
            return
        self._first_after_activation = False
        self._last_handled = value
        self.get_logger().info(f'product_in_place={value}')
        if value:
            reader.on_product_arrived(self.get_clock().now())
            runner.submit(ProcessTypeEnum.INJECTION, process.inject)
        else:
            reader.on_product_left()
            runner.submit(ProcessTypeEnum.EJECTION, process.eject)
    # endregion trigger

    # region system failure self-transition
    def _on_system_failure(self, reason: str) -> None:
        """Schedule the deferred deactivate -> cleanup (any thread, idempotent)."""
        with self._fatal_lock:
            if self._fatal_pending:
                return
            self._fatal_pending = True
        self.get_logger().error(f'System failure, transitioning to unconfigured: {reason}')
        self._fatal_guard.trigger()

    def _on_fatal_guard(self) -> None:
        """Run the self-transition in the executor."""
        with self._fatal_lock:
            if not self._fatal_pending:
                return
        if not self._is_active:
            self.get_logger().info('System failure reported but node is no longer active')
            return
        result = self.trigger_deactivate()
        self.get_logger().info(f'Self-transition deactivate: {result}')
        result = self.trigger_cleanup()
        self.get_logger().info(f'Self-transition cleanup: {result}')
    # endregion system failure self-transition


def spin_node(node: LifecycleNode) -> None:
    """Spin ``node`` on a multi-threaded executor until shutdown."""
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
