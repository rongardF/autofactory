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
"""Gazebo world manager — spawn and remove models in a live Gazebo world.

The manager talks to Gazebo entirely through ``ros_gz_bridge``; it never imports
any ``gz.*`` binding. Models are spawned via the world ``create`` service and
removed via the world ``remove`` service.

A model is spawned at a caller-supplied pose that is expressed relative to a TF
frame (``link_name``), so the pose is first composed with the ``world ->
link_name`` transform before being handed to Gazebo. Welding is the
responsibility of the caller-supplied SDF: a detachable joint declared in the
SDF is attached when the model is spawned.
"""

from __future__ import annotations

import time
from uuid import uuid4

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.client import Client as ServiceClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point, Pose, Quaternion
# NOTE: TransformException is re-exported by tf2_ros from the compiled tf2_py
# C-extension via a wildcard import, so Pylance/Pyright cannot resolve it
# statically even though it exists at runtime.
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException  # type: ignore[attr-defined]

from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity

from endtools.exception.world_manager_exception import ModelSpawnError
from endtools.utils.transformations import rpy_to_quaternion


class GazeboWorldManager():

    # Frame all spawned entities are positioned against.
    WORLD_FRAME_ID = 'world'

    def __init__(
        self,
        node: Node,
        world_name: str,
        tf_timeout_sec: float = 3.0,
        service_timeout_sec: float = 5.0,
    ):
        super().__init__()
        self._tf_timeout_sec = tf_timeout_sec
        self._service_timeout_sec = service_timeout_sec
        self._node = node
        self._logger = node.get_logger()
        self._world_name = world_name

        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_callback_group = ReentrantCallbackGroup()

        # Link each spawned model's pose was resolved against, keyed by model ID.
        self._attached_link_lookup: dict[str, str] = {}

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

    @staticmethod
    def _multiply_quaternions(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        """Compose two quaternions with the Hamilton product ``first * second``.

        :param first: Left-hand ``(x, y, z, w)`` quaternion.
        :param second: Right-hand ``(x, y, z, w)`` quaternion.
        :returns: The composed ``(x, y, z, w)`` quaternion.
        """
        x1, y1, z1, w1 = first
        x2, y2, z2, w2 = second
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _rotate_vector(
        quaternion: tuple[float, float, float, float],
        vector: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Rotate a vector by a quaternion.

        :param quaternion: Rotation as a ``(x, y, z, w)`` quaternion.
        :param vector: Vector ``(x, y, z)`` to rotate.
        :returns: The rotated vector ``(x, y, z)``.
        """
        qx, qy, qz, qw = quaternion
        vx, vy, vz = vector

        # t = 2 * (q_vector x v); result = v + qw * t + q_vector x t
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def _lookup_pose_in_world(self, pose: tuple, frame_id: str) -> Pose | None:
        """Express a frame-relative pose in the world frame.

        ``pose`` is a cartesian ``(x, y, z, roll, pitch, yaw)`` tuple (m, rad)
        expressed in ``frame_id``. It is composed with the looked-up
        ``world -> frame_id`` transform.

        :param pose: Pose relative to ``frame_id`` as ``(x, y, z, roll, pitch, yaw)``.
        :param frame_id: Frame the given pose is expressed in.
        :returns: The pose expressed in the world frame, or ``None`` on lookup
            failure or on a malformed ``pose``.
        """
        try:
            x, y, z, roll, pitch, yaw = pose
        except (TypeError, ValueError):
            self._logger.error(
                f"Invalid pose {pose!r}; expected (x, y, z, roll, pitch, yaw)"
            )
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self.WORLD_FRAME_ID,
                frame_id,
                Time(),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as exc:
            self._logger.error(
                f"Failed to look up '{frame_id}' in '{self.WORLD_FRAME_ID}': {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        frame_orientation = (rotation.x, rotation.y, rotation.z, rotation.w)

        offset_x, offset_y, offset_z = self._rotate_vector(
            frame_orientation, (float(x), float(y), float(z))
        )
        world_x, world_y, world_z, world_w = self._multiply_quaternions(
            frame_orientation, rpy_to_quaternion(float(roll), float(pitch), float(yaw))
        )

        world_pose = Pose()
        world_pose.position = Point(
            x=translation.x + offset_x,
            y=translation.y + offset_y,
            z=translation.z + offset_z,
        )
        world_pose.orientation = Quaternion(
            x=world_x, y=world_y, z=world_z, w=world_w
        )
        return world_pose

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

    def spawn_model(
        self,
        sdf_content: str,
        pose: tuple,
        link_name: str,
        model_id: str|None = None
    ) -> str:
        """Spawn a model at a pose relative to ``link_name``.

        :raises ModelSpawnError: If the pose could not be resolved or the Gazebo
            world ``create`` service did not confirm the spawn.
        """
        if model_id is None:
            mod_id = f"model_{uuid4()}"
        else:
            mod_id = model_id

        try:
            spawn_pose_in_world = self._lookup_pose_in_world(pose, link_name)
            if spawn_pose_in_world is None:
                raise ModelSpawnError(
                    f'Failed to look up world pose of link {link_name} for model {mod_id}'
                )
            factory = EntityFactory()
            factory.name = mod_id
            factory.sdf = sdf_content
            factory.pose = spawn_pose_in_world
            factory.relative_to = self.WORLD_FRAME_ID

            request = SpawnEntity.Request()
            request.entity_factory = factory
            response = self._call(self._create_client, request, f'spawn model {mod_id}')
            if response is None or not response.success:
                raise ModelSpawnError(f'Failed to spawn model {mod_id} in Gazebo world')

        except Exception as e:
            self._logger.error(f'Error spawning model {mod_id}: {e}')
            raise ModelSpawnError(f'Error spawning model {mod_id}: {e}') from e

        self._attached_link_lookup[mod_id] = link_name

        return mod_id

    def delete_model(self, model_id: str) -> bool:
        try:
            entity = Entity()
            entity.name = model_id
            entity.type = Entity.MODEL

            request = DeleteEntity.Request()
            request.entity = entity
            response = self._call(self._remove_client, request, f'remove model {model_id}')
            if response is None or not response.success:
                self._logger.error(f'Failed to remove model {model_id} from Gazebo world')
                return False

        except Exception as e:
            self._logger.error(f'Error deleting model {model_id}: {e}')
            return False

        self._attached_link_lookup.pop(model_id, None)

        return True

    def update_pose(self, model_id: str, pose: tuple, link_name: str|None = None) -> bool:
        raise NotImplementedError()