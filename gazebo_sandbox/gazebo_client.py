#!/usr/bin/env python3
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
"""Minimal ROS 2 client for manipulating models in a running Gazebo world.

This client talks to Gazebo purely through ``ros_gz_bridge`` — it never imports
any ``gz.*`` Python binding. Run the bridge separately so that the following
Gazebo world services and topics are exposed on the ROS graph:

Services (gz world service  ->  ROS type)::

    /world/<world>/create     ros_gz_interfaces/srv/SpawnEntity
    /world/<world>/set_pose   ros_gz_interfaces/srv/SetEntityPose
    /world/<world>/remove     ros_gz_interfaces/srv/DeleteEntity

Topics (gz topic  ->  ROS type)::

    /model/<name>/pose        tf2_msgs/msg/TFMessage   (gz.msgs.Pose_V)
    /mating/attach            std_msgs/msg/Empty       (gz.msgs.Empty)
    /mating/detach            std_msgs/msg/Empty       (gz.msgs.Empty)
    /mating/state             std_msgs/msg/String      (gz.msgs.StringMsg)

The model pose comes from a ``gz-sim-pose-publisher-system`` plugin attached to
the model (see ``models/tool_side/model.sdf``). Unlike the world
``/world/<world>/pose/info`` topic — whose ``gz.msgs.Pose`` carries the entity
name only in the ``name`` field, which the bridge drops — PosePublisher writes
``frame_id``/``child_frame_id`` into the message header data, so the bridged
``TFMessage`` keeps the model name in ``child_frame_id``.

Example ``ros_gz_bridge`` invocation exposing the services and topics above::

    ros2 run ros_gz_bridge parameter_bridge \\
        /world/spawn/create@ros_gz_interfaces/srv/SpawnEntity \\
        /world/spawn/set_pose@ros_gz_interfaces/srv/SetEntityPose \\
        /world/spawn/remove@ros_gz_interfaces/srv/DeleteEntity \\
        /model/mating_box/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
        /mating/attach@std_msgs/msg/Empty]gz.msgs.Empty \\
        /mating/detach@std_msgs/msg/Empty]gz.msgs.Empty \\
        /mating/state@std_msgs/msg/String[gz.msgs.StringMsg

Important constraints (verified against Gazebo Harmonic / gz-sim 8.11.0):

* ``SpawnEntity`` returns only ``success`` (a ``bool``); it does NOT return a
  numeric entity id. The stable handle for a model is therefore its **name**,
  which the ``ros_gz_interfaces/Entity`` message accepts in place of an id for
  ``set_pose``/``remove``. All methods below use the model name as the
  ``entity_id``.
* The demo world defines a single, global ``DetachableJoint`` wired to the fixed
  ``/mating/*`` topics, so ``attach_model``/``detach_model`` act on that one
  joint. The ``entity_id`` argument is kept for API symmetry.
"""

from threading import Lock
from enum import Enum
from typing import Optional
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from pydantic import BaseModel, ConfigDict, Field
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage

from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity


class AttachStateEnum(str, Enum):
    """Latest observed state of the world's detachable (mating) joint."""

    ATTACHED = 'attached'
    DETACHED = 'detached'
    UNKNOWN = 'unknown'


class CachedPoseDTO(BaseModel):
    """A single pose reading cached from the bridged ``pose/info`` topic.

    ``stamp_ns`` is the Gazebo-published header stamp flattened to nanoseconds
    (sim time), used to decide whether a reading is newer than a given moment.
    """

    model_config = ConfigDict(frozen=True)

    stamp_ns: int = Field(
        description='Header stamp in nanoseconds (sim time) of this reading.',
    )
    frame_id: str = Field(
        default='world',
        description='Parent/reference frame the pose is expressed in.',
    )
    position: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description='Cartesian position (x, y, z) in metres.',
    )
    orientation: tuple[float, float, float, float] = Field(
        default=(0.0, 0.0, 0.0, 1.0),
        description='Orientation quaternion (x, y, z, w).',
    )


class ModelDataDTO(BaseModel):
    """Snapshot of a spawned model's pose and mating state.

    ``position`` and ``orientation`` are expressed relative to ``frame_id`` (the
    parent frame reported on the bridged ``pose/info`` transform, usually the
    world frame).
    """

    model_config = ConfigDict(frozen=True)

    name: str = Field(description='Model name used as the entity handle.')
    frame_id: str = Field(
        default='world',
        description='Parent/reference frame the pose is expressed in.',
    )
    position: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description='Cartesian position (x, y, z) in metres.',
    )
    orientation: tuple[float, float, float, float] = Field(
        default=(0.0, 0.0, 0.0, 1.0),
        description='Orientation quaternion (x, y, z, w).',
    )
    attach_state: AttachStateEnum = Field(
        default=AttachStateEnum.UNKNOWN,
        description='Most recent mating-joint state seen on /mating/state.',
    )


class GazeboModelClient(Node):
    """ROS 2 node that adds, moves, welds and removes Gazebo models.

    The node owns synchronous service clients (add/move/remove) plus latching
    subscriptions that cache the newest pose and mating state so that
    :meth:`get_model_data` can answer without blocking on the network.
    """

    def __init__(self, node_name: str = 'gazebo_model_client') -> None:
        super().__init__(node_name)

        # --- parameters -------------------------------------------------
        self.declare_parameter('world_name', 'spawn')
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('attach_topic', '/mating/attach')
        self.declare_parameter('detach_topic', '/mating/detach')
        self.declare_parameter('state_topic', '/mating/state')
        self.declare_parameter('pose_topic', '/model/mating_box/pose')

        world = str(self.get_parameter('world_name').value)
        self._service_timeout = float(
            self.get_parameter('service_timeout_sec').value  # type: ignore[arg-type]
        )
        attach_topic = str(self.get_parameter('attach_topic').value)
        detach_topic = str(self.get_parameter('detach_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        # --- service clients -------------------------------------------
        self._create_cli = self.create_client(
            SpawnEntity, f'/world/{world}/create'
        )
        self._set_pose_cli = self.create_client(
            SetEntityPose, f'/world/{world}/set_pose'
        )
        self._remove_cli = self.create_client(
            DeleteEntity, f'/world/{world}/remove'
        )

        # --- attach / detach publishers --------------------------------
        self._attach_pub = self.create_publisher(Empty, attach_topic, 10)
        self._detach_pub = self.create_publisher(Empty, detach_topic, 10)

        # --- cached state from subscriptions ---------------------------
        # Guards _pose_cache and _attach_state so the subscription callbacks
        # (which may run under a MultiThreadedExecutor) never race with reads.
        self._lock = Lock()
        # child_frame_id (model name) -> latest cached pose reading
        self._pose_cache: dict[str, CachedPoseDTO] = {}
        self._attach_state = AttachStateEnum.UNKNOWN

        self.create_subscription(
            TFMessage, pose_topic, self._on_pose_info, 10
        )
        self.create_subscription(
            String, state_topic, self._on_state, 10
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def add_model(
        self,
        uri: str,
        name: str,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    ) -> Optional[str]:
        """Spawn a model from a URI (e.g. ``model://tool_side/``).

        Returns the model ``name`` (the entity handle) on success, or ``None``
        on failure. The bridged ``SpawnEntity`` response carries no numeric id,
        so the name is the stable handle used by every other method here.
        """
        factory = EntityFactory()
        factory.name = name
        factory.sdf_filename = uri
        factory.pose = self._make_pose(position, orientation)
        factory.relative_to = 'world'

        request = SpawnEntity.Request()
        request.entity_factory = factory

        response = self._call(self._create_cli, request, 'create')
        if response is None or not response.success:
            self.get_logger().error(f"add_model('{name}') failed")
            return None
        return name

    def move_model(
        self,
        entity_id: str,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    ) -> bool:
        """Teleport a model to an absolute world pose. Returns success."""
        request = SetEntityPose.Request()
        request.entity = self._make_entity(entity_id)
        request.pose = self._make_pose(position, orientation)

        response = self._call(self._set_pose_cli, request, 'set_pose')
        if response is None or not response.success:
            self.get_logger().error(f"move_model('{entity_id}') failed")
            return False
        return True

    def attach_model(
        self, entity_id: str, timeout_sec: float = 5.0
    ) -> bool:
        """Weld the mating joint and block until it is confirmed attached.

        Acts on the world's single global detachable joint; ``entity_id`` is
        accepted for API symmetry but does not select the joint. Publishes the
        attach request, then spins until ``/mating/state`` reports ``attached``.
        Returns ``True`` once confirmed, or ``False`` if the state does not turn
        ``attached`` within ``timeout_sec`` (defaults to ``confirm_timeout_sec``).
        """
        self._attach_pub.publish(Empty())
        self.get_logger().info(f"attach requested (context '{entity_id}')")
        if not self._wait_for_attach_state(
            AttachStateEnum.ATTACHED, timeout_sec
        ):
            self.get_logger().error(
                f"attach_model('{entity_id}') not confirmed within window"
            )
            return False
        return True

    def detach_model(
        self, entity_id: str, timeout_sec: float = 5.0
    ) -> bool:
        """Release the mating joint and block until it is confirmed detached.

        See :meth:`attach_model` for scope. Publishes the detach request, then
        spins until ``/mating/state`` reports ``detached``. Returns ``True`` once
        confirmed, or ``False`` if the state does not turn ``detached`` within
        ``timeout_sec`` (defaults to ``confirm_timeout_sec``).
        """
        self._detach_pub.publish(Empty())
        self.get_logger().info(f"detach requested (context '{entity_id}')")
        if not self._wait_for_attach_state(
            AttachStateEnum.DETACHED, timeout_sec
        ):
            self.get_logger().error(
                f"detach_model('{entity_id}') not confirmed within window"
            )
            return False
        return True

    def remove_model(self, entity_id: str) -> bool:
        """Delete a spawned model from the world. Returns success."""
        request = DeleteEntity.Request()
        request.entity = self._make_entity(entity_id)

        response = self._call(self._remove_cli, request, 'remove')
        if response is None or not response.success:
            self.get_logger().error(f"remove_model('{entity_id}') failed")
            return False
        with self._lock:
            self._pose_cache.pop(entity_id, None)
        return True

    def get_model_data(
        self, entity_id: str, timeout_sec: float = 0.5
    ) -> Optional[ModelDataDTO]:
        """Return the model's pose, frame and mating state if freshly updated.

        Records the stamp of the currently cached pose for ``entity_id`` as a
        baseline, spins for ``timeout_sec`` seconds to let new ``pose/info``
        messages arrive, then returns the cached pose only if a reading with a
        newer stamp arrived during the window. Returns ``None`` if no pose for
        ``entity_id`` has ever been observed or none newer than the baseline
        arrived within the window.

        The baseline is the previous message stamp (Gazebo sim time), so the
        freshness comparison is between two pose stamps in the same clock
        domain — it does not depend on ``use_sim_time`` or a bridged ``/clock``.
        """
        with self._lock:
            cached = self._pose_cache.get(entity_id)
            baseline_ns = cached.stamp_ns if cached is not None else -1

        end_ns = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self.get_clock().now().nanoseconds < end_ns:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                current = self._pose_cache.get(entity_id)
            if current is not None and current.stamp_ns > baseline_ns:
                break

        with self._lock:
            cached = self._pose_cache.get(entity_id)
            attach_state = self._attach_state

        if cached is None or cached.stamp_ns <= baseline_ns:
            self.get_logger().warn(
                f"get_model_data('{entity_id}'): no pose newer than "
                f'{timeout_sec:.3f}s window'
            )
            return None

        return ModelDataDTO(
            name=entity_id,
            frame_id=cached.frame_id,
            position=cached.position,
            orientation=cached.orientation,
            attach_state=attach_state,
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_pose_info(self, msg: TFMessage) -> None:
        with self._lock:
            for tf in msg.transforms:
                stamp = tf.header.stamp
                stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                t = tf.transform.translation
                r = tf.transform.rotation
                self._pose_cache[tf.child_frame_id] = CachedPoseDTO(
                    stamp_ns=stamp_ns,
                    frame_id=tf.header.frame_id,
                    position=(t.x, t.y, t.z),
                    orientation=(r.x, r.y, r.z, r.w),
                )

    def _on_state(self, msg: String) -> None:
        value = msg.data.strip().lower()
        with self._lock:
            if value == AttachStateEnum.ATTACHED.value:
                self._attach_state = AttachStateEnum.ATTACHED
            elif value == AttachStateEnum.DETACHED.value:
                self._attach_state = AttachStateEnum.DETACHED

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _wait_for_attach_state(
        self, desired: AttachStateEnum, timeout_sec: float = 5.0
    ) -> bool:
        """Spin until the cached mating state equals ``desired``.

        Returns ``True`` as soon as ``/mating/state`` reports ``desired``, or
        ``False`` if it has not by the time ``timeout_sec`` elapses.
        used.
        """
        end_ns = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self.get_clock().now().nanoseconds < end_ns:
            with self._lock:
                if self._attach_state == desired:
                    return True
            rclpy.spin_once(self, timeout_sec=0.05)
        with self._lock:
            return self._attach_state == desired

    def _call(self, client, request, label: str):
        """Call a service synchronously with a bounded wait. Returns response
        or ``None`` on timeout/unavailability."""
        if not client.wait_for_service(timeout_sec=self._service_timeout):
            self.get_logger().error(
                f"service '{label}' unavailable "
                f"(is ros_gz_bridge running?)"
            )
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._service_timeout
        )
        if not future.done():
            self.get_logger().error(f"service '{label}' call timed out")
            return None
        return future.result()

    def _make_pose(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> Pose:
        pose = Pose()
        pose.position = Point(x=position[0], y=position[1], z=position[2])
        pose.orientation = Quaternion(
            x=orientation[0], y=orientation[1],
            z=orientation[2], w=orientation[3],
        )
        return pose

    def _make_entity(self, name: str) -> Entity:
        entity = Entity()
        entity.name = name
        entity.type = Entity.MODEL
        return entity


def main() -> None:
    """Small demo: spawn, move, weld, inspect, release and remove a model."""
    rclpy.init()
    client = GazeboModelClient()
    logger = client.get_logger()
    try:
        logger.info("Starting")
        sleep(10.0)
        handle = client.add_model(
            uri='model://tool_side/',
            name='mating_box',
            position=(0.0, -0.3, 0.0),
        )
        if handle is None:
            return

        logger.info(f"spawned model '{handle}'")
        sleep(5.0)

        client.detach_model(handle)
        logger.info(f"Model detached '{handle}'")
        sleep(5.0)

        client.move_model(handle, position=(0.0, -0.15, 0.0))
        logger.info(f"moved model '{handle}'")
        sleep(5.0)
        client.attach_model(handle)
        logger.info(f"model attached '{handle}'")
        sleep(5.0)

        data = client.get_model_data(handle)
        if data is not None:
            logger.info(f'model data: {data.model_dump()}')

        sleep(5.0)
        client.detach_model(handle)
        logger.info(f"Model detached '{handle}'")
        sleep(5.0)

        client.remove_model(handle)
        logger.info(f"Model removed '{handle}'")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
