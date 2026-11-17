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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
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
"""Adapt workplace planning-scene operations to the shared MoveIt client."""

import threading
from collections.abc import Iterable
from pathlib import Path

from geometry_msgs.msg import Pose

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive

from station_workplace.enums.constants import (
    CARRIER_OBJECT_ID,
    PRODUCT_OBJECT_ID,
    SAFETY_OBJECT_ID,
)
from station_workplace.models.common_node_config_dto import (
    SafetyObjectConfigDTO,
)

from world_manager.client import MoveIt2WorldClient
from world_manager.exceptions import WorldManagerError
from world_manager.models import (
    AllowedCollisionPairDTO,
    PoseDTO,
    SceneObjectDTO,
    SolidPrimitiveDTO,
    WorldChangesDTO,
)


class PlanningSceneAdapter:
    """Translate workplace operations into typed world-manager requests."""

    def __init__(
        self,
        node: Node,
        attach_link: str,
        collision_allowed_link: str,
        safety_size: SafetyObjectConfigDTO,
        timeout_sec: float,
        callback_group: CallbackGroup,
    ) -> None:
        """Store geometry settings and client construction context."""
        self._node = node
        self._attach_link = attach_link
        self._collision_allowed_link = collision_allowed_link
        self._safety_size = safety_size
        self._timeout_sec = timeout_sec
        self._callback_group = callback_group
        self._client: MoveIt2WorldClient | None = None

    def activate(self) -> None:
        """Create clients; fail if the shared manager is unavailable."""
        client = MoveIt2WorldClient(
            self._node,
            self._callback_group,
            timeout_sec=self._timeout_sec,
        )
        try:
            client.list_objects()
        except Exception:
            client.destroy()
            raise
        self._client = client

    def deactivate(self) -> None:
        """Destroy the service clients without changing the planning scene."""
        if self._client is not None:
            self._client.destroy()
            self._client = None

    def get_present_object_ids(self, cancel: threading.Event) -> set[str]:
        """Return all world and attached planning-scene object IDs."""
        return {
            item.object_id
            for item in self._require_client().list_objects(cancel)
        }

    def attach_safety_object(self, cancel: threading.Event) -> None:
        """Attach the safety box and allow contact with the configured link."""
        x, y, z = self._safety_size.x, self._safety_size.y, self._safety_size.z
        safety = SceneObjectDTO(
            object_id=SAFETY_OBJECT_ID,
            primitives=(
                SolidPrimitiveDTO(
                    type=SolidPrimitive.BOX,
                    dimensions=(x, y, z),
                ),
            ),
            primitive_poses=(
                PoseDTO(x=-x / 2.0, z=z / 2.0),
            ),
            pose=PoseDTO(frame_id=self._attach_link),
            attach_link=self._attach_link,
            touch_links=(self._collision_allowed_link,),
        )
        self._require_client().apply(
            WorldChangesDTO(add_or_replace=(safety,)),
            cancel,
        )
        self._node.get_logger().info(
            f"Attached '{SAFETY_OBJECT_ID}' to '{self._attach_link}'"
        )

    def attach_product_and_carrier(
        self,
        product_mesh_path: Path,
        carrier_mesh_path: Path,
        product_pose: Pose,
        cancel: threading.Event,
    ) -> None:
        """Replace the safety box and add both meshes in a single diff."""
        carrier = SceneObjectDTO(
            object_id=CARRIER_OBJECT_ID,
            mesh_path=str(carrier_mesh_path),
            mesh_file_type='stl',
            pose=PoseDTO(frame_id=self._attach_link),
            attach_link=self._attach_link,
            touch_links=(self._collision_allowed_link,),
        )
        product = SceneObjectDTO(
            object_id=PRODUCT_OBJECT_ID,
            mesh_path=str(product_mesh_path),
            mesh_file_type='stl',
            pose=self._pose_to_dto(product_pose, self._attach_link),
            attach_link=self._attach_link,
        )
        self._require_client().apply(
            WorldChangesDTO(
                add_or_replace=(carrier, product),
                remove_ids=(SAFETY_OBJECT_ID,),
                acm_changes=(
                    AllowedCollisionPairDTO(
                        name_a=PRODUCT_OBJECT_ID,
                        name_b=CARRIER_OBJECT_ID,
                        allowed=True,
                    ),
                ),
            ),
            cancel,
        )
        self._node.get_logger().info(
            f"Attached '{CARRIER_OBJECT_ID}' and '{PRODUCT_OBJECT_ID}' "
            f"to '{self._attach_link}'"
        )

    def remove_objects(
        self,
        object_ids: Iterable[str],
        cancel: threading.Event,
    ) -> None:
        """Remove requested objects; absent IDs are ignored by the manager."""
        remove_ids = tuple(dict.fromkeys(object_ids))
        if not remove_ids:
            return
        self._require_client().apply(
            WorldChangesDTO(remove_ids=remove_ids),
            cancel,
        )

    @staticmethod
    def _pose_to_dto(pose: Pose, frame_id: str) -> PoseDTO:
        """Convert a ROS pose into the configured attach-link frame."""
        return PoseDTO(
            frame_id=frame_id,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            qx=pose.orientation.x,
            qy=pose.orientation.y,
            qz=pose.orientation.z,
            qw=pose.orientation.w,
        )

    def _require_client(self) -> MoveIt2WorldClient:
        """Return the active client or raise an explicit lifecycle error."""
        if self._client is None:
            raise WorldManagerError(
                'workplace planning-scene adapter is not active'
            )
        return self._client
