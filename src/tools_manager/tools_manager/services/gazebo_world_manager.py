# Copyright (c) 2026, Tools Manager Contributors
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
"""Gazebo world manager — spawn, remove and re-mate tool models in a live world.

The manager talks to Gazebo entirely through ``ros_gz_bridge``; it never imports
any ``gz.*`` binding. Models are spawned via the world ``create`` service and
removed via ``remove``. Each spawned tool carries detachable-joint plugins whose
attach/detach requests and state feedback are bridged onto per-link, per-model
topics of the form ``/<link_name>/<model_id>/{attach,detach,state}``.

The link a model is welded to is supplied by the caller and the manager keeps an 
internal record of the link each model
is currently attached to so transfers only need the destination link.
"""

from __future__ import annotations

import threading
import time

import xacro

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.client import Client as ServiceClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.interface.world_manager import WorldManager
from tools_manager.exception.world_manager_exception import (
    ModelAttachError,
    ModelDeleteError,
    ModelSpawnError,
)



class GazeboWorldManager(WorldManager):
    """Manage tool models inside a running Gazebo world over ``ros_gz_bridge``.

    A tool is spawned at the world pose of a caller-supplied link and welded to
    that link via a detachable joint. Transferring a tool between links attaches
    the destination joint first and only then detaches the source joint, so the
    model is never held by zero joints. The manager tracks the link each model is
    currently attached to so ``attach_to_link`` only needs the destination link.
    """

    def __init__(
        self,
        node: Node,
        world_name: str,
        station_model_name: str,
        tool_mount_link: str = 'tool_mount_tcp',
        service_timeout_sec: float = 5.0,
        tf_timeout_sec: float = 5.0,
        confirm_timeout_sec: float = 5.0,
    ) -> None:
        """Create the Gazebo service clients and the TF listener.

        :param node: Node used to create clients/publishers and access logger/clock.
        :param world_name: Name of the Gazebo world (used in the service names).
        :param station_model_name: Name of the station model in the world.
        :param tool_mount_link: Link a tool is welded to when mounted.
        :param service_timeout_sec: Timeout for world service availability/results.
        :param tf_timeout_sec: Timeout for slot-frame TF lookups.
        :param confirm_timeout_sec: Timeout for detachable-joint state confirmation.
        """
        super().__init__()

        self._node = node
        self._logger = node.get_logger()
        self._world_name = world_name
        self._station_model_name = station_model_name
        self._tool_mount_link = tool_mount_link
        self._service_timeout_sec = service_timeout_sec
        self._tf_timeout_sec = tf_timeout_sec
        self._confirm_timeout_sec = confirm_timeout_sec

        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_callback_group = ReentrantCallbackGroup()

        self._attached_link_lookup = {}

        self._create_client = node.create_client(
            SpawnEntity,
            f'/world/{world_name}/create',
            callback_group=self._service_callback_group,
        )
        self._remove_client = node.create_client(
            DeleteEntity,
            f'/world/{world_name}/remove',
            callback_group=self._service_callback_group,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    def _generate_model_path(self, tool: ToolInfoDto) -> str:
        """Generate the filesystem path to a tool's XACRO model description.

        :param tool: Tool information.
        :returns: Filesystem path to the XACRO model description.
        :raises NotImplementedError: Always; subclasses must implement this.
        """
        raise NotImplementedError('subclass must implement _generate_model_path()')

    def _transfer(self, model_id: str, source_link: str, target_link: str) -> bool:
        """Transfer a model's weld from one link to another.

        The destination joint is attached first and only then is the source joint
        detached, so the model is never held by zero joints.

        :param model_id: Unique model ID (equal to the tool serial number).
        :param source_link: Link the model is currently welded to.
        :param target_link: Link to weld the model to.
        :returns: ``True`` on success, ``False`` otherwise.
        """
        if not self._apply_joint_action(target_link, model_id, attach=True):
            return False
        return self._apply_joint_action(source_link, model_id, attach=False)

    def _generate_sdf_content(
        self, model_path: str, model_name: str, tool_rack_link: str
    ) -> str:
        """Expand a XACRO description into an SDF string via the xacro API.

        The tool carries two detachable joints: one to the tool-mount link and
        one to ``tool_rack_link`` (the tool's rack slot link).

        :param model_path: Filesystem path to the XACRO model description.
        :param model_name: Model name (equal to the model ID / tool serial number).
        :param tool_rack_link: Link the tool's rack joint targets.
        :returns: The expanded SDF document as a string.
        """
        document = xacro.process_file(
            model_path,
            mappings={
                'model_name': model_name,
                'station_model_name': self._station_model_name,
                'tool_mount_link': self._tool_mount_link,
                'tool_rack_link': tool_rack_link,
            },
        )
        return document.toxml()  # type: ignore

    def _lookup_pose_in_world(self, frame_id: str) -> Pose | None:
        """Look up the world pose of a TF frame.

        :param frame_id: Frame whose pose in the world frame is requested.
        :returns: The frame's pose expressed in the world frame, or ``None`` on
            lookup failure.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                'world',
                frame_id,
                Time(),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as exc:
            self._logger.error(f"Failed to look up '{frame_id}' in '{'world'}': {exc}")
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = Pose()
        pose.position = Point(x=translation.x, y=translation.y, z=translation.z)
        pose.orientation = Quaternion(
            x=rotation.x, y=rotation.y, z=rotation.z, w=rotation.w
        )
        return pose

    def _detach_best_effort(self, link_name: str, model_id: str) -> None:
        """Request a detach without failing when no confirmation arrives.

        Used before model removal, where one of the two joints is already
        detached and therefore emits no state event.

        :param link_name: Link whose detachable joint should be detached.
        :param model_id: Unique model ID (equal to the tool serial number).
        """
        if not self._apply_joint_action(link_name, model_id, attach=False):
            self._logger.warn(
                f"Detach of '{model_id}' on '{link_name}' not confirmed "
                f'(joint may already be detached)'
            )

    def _apply_joint_action(
        self, link_name: str, model_id: str, attach: bool
    ) -> bool:
        """Attach or detach a model's detachable joint and confirm via state.

        Subscribes to the joint's ``state`` topic before publishing the request
        so the resulting (event-only, non-latched) state message is not missed,
        then blocks until the desired state is observed or the timeout elapses.

        :param link_name: Link whose detachable joint is acted upon.
        :param model_id: Unique model ID (equal to the tool serial number).
        :param attach: ``True`` to attach, ``False`` to detach.
        :returns: ``True`` if the desired state was confirmed, ``False`` otherwise.
        """
        desired = 'attached' if attach else 'detached'
        verb = 'attach' if attach else 'detach'
        confirmed = threading.Event()

        def _on_state(msg: String) -> None:
            if msg.data.strip().lower() == desired:
                confirmed.set()

        state_topic = f'/{link_name}/{model_id}/state'
        action_topic = f'/{link_name}/{model_id}/{verb}'

        subscription = self._node.create_subscription(
            String,
            state_topic,
            _on_state,
            10,
            callback_group=self._state_callback_group,
        )
        publisher = self._node.create_publisher(Empty, action_topic, 10)
        try:
            if not self._wait_for_subscriber(publisher):
                self._logger.error(f"No subscriber matched on '{action_topic}'")
                return False

            publisher.publish(Empty())
            if not confirmed.wait(timeout=self._confirm_timeout_sec):
                self._logger.error(
                    f"'{verb}' of '{model_id}' on '{link_name}' not confirmed "
                    f'within {self._confirm_timeout_sec:.1f}s'
                )
                return False
            return True
        finally:
            self._node.destroy_publisher(publisher)
            self._node.destroy_subscription(subscription)

    def _wait_for_subscriber(self, publisher) -> bool:
        """Block until the publisher has at least one matched subscriber.

        The attach/detach topics are consumed by ``ros_gz_bridge``; publishing
        before it has matched would drop the request.

        :param publisher: Publisher to wait on.
        :returns: ``True`` once a subscriber is matched, ``False`` on timeout.
        """
        deadline = time.monotonic() + self._service_timeout_sec
        while publisher.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return True

    def _call(self, client: ServiceClient, request, description: str):
        """Call a Gazebo world service and wait for the result.

        Uses ``call_async`` and polls the future so this can run on an executor
        thread without nested spinning.

        :param client: Service client to call.
        :param request: Request message.
        :param description: Human-readable operation name for logging.
        :returns: The service response, or ``None`` on unavailability/timeout.
        """
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self._logger.error(f'Service not available for: {description}')
            return None

        future = client.call_async(request)
        deadline = time.monotonic() + self._service_timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                self._logger.error(f'Service call timed out for: {description}')
                return None
            time.sleep(0.01)

        return future.result()

    def spawn_model(self, tool: ToolInfoDto, link_name: str):
        try:
            model_id = tool.tool_sn
            model_path = self._generate_model_path(tool)
            slot_link = tool.tool_attached_frame
            sdf = self._generate_sdf_content(model_path, model_id, slot_link)

            pose = self._lookup_pose_in_world(link_name)
            if pose is None:
                raise ModelSpawnError(
                    f'Failed to look up world pose of link {link_name} for model {model_id}'
                )

            factory = EntityFactory()
            factory.name = model_id
            factory.sdf = sdf
            factory.pose = pose
            factory.relative_to = 'world'

            request = SpawnEntity.Request()
            request.entity_factory = factory
            response = self._call(self._create_client, request, f'spawn model {model_id}')
            if response is None or not response.success:
                raise ModelSpawnError(f'Failed to spawn model {model_id} in Gazebo world')

            # Both detachable joints start attached; detach the redundant one so
            # the tool is welded to ``link_name`` only.
            redundant_link = slot_link if link_name == self._tool_mount_link else self._tool_mount_link
            if not self._apply_joint_action(redundant_link, model_id, attach=False):
                raise ModelSpawnError(
                    f'Model {model_id} spawned but detach of {redundant_link} not confirmed'
                )

            self._attached_link_lookup[model_id] = link_name
        except Exception as exc:
            self._logger.error(f'Failed to spawn model {tool.tool_sn}: {exc}')
            raise ModelSpawnError(
                f'Failed to spawn model {tool.tool_sn} in Gazebo world: {exc}'
            ) from exc

    def delete_model(self, model_id: str):
        try:
            source_link = self._attached_link_lookup.get(model_id, None)
            if source_link is not None:
                self._detach_best_effort(source_link, model_id)

            entity = Entity()
            entity.name = model_id
            entity.type = Entity.MODEL

            request = DeleteEntity.Request()
            request.entity = entity
            response = self._call(self._remove_client, request, f'remove model {model_id}')
            if response is None or not response.success:
                raise ModelDeleteError(f'Failed to remove model {model_id} from Gazebo world')

            self._attached_link_lookup.pop(model_id, None)
        except Exception as exc:
            self._logger.error(f'Failed to delete model {model_id}: {exc}')
            raise ModelDeleteError(
                f'Failed to delete model {model_id} in Gazebo world: {exc}'
            ) from exc

    def attach_to_link(self, model_id: str, link_name: str):
        try:
            source_link = self._attached_link_lookup.get(model_id, None)
            if source_link is None:
                self._logger.error(f'Model {model_id} is not currently attached to any link')
                raise ModelAttachError(f'Model {model_id} is not currently attached to any link')
            if source_link == link_name:
                return
            if not self._transfer(model_id, source_link, link_name):
                raise ModelAttachError(f'Failed to attach model {model_id} to link {link_name}')

            self._attached_link_lookup[model_id] = link_name
        except Exception as exc:
            self._logger.error(f'Failed to attach model {model_id} to link {link_name}: {exc}')
            raise ModelAttachError(
                f'Failed to attach model {model_id} to link {link_name}: {exc}'
            ) from exc

    