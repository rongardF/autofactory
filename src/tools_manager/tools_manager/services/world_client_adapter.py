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
"""Adapt tool lifecycle operations to the shared world-manager clients."""

from itertools import combinations
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from geometry_msgs.msg import PoseStamped

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO

from world_manager.client import GazeboWorldClient, MoveIt2WorldClient
from world_manager.exceptions import WorldManagerError
from world_manager.models import (
    AllowedCollisionPairDTO,
    ModelSpecDTO,
    PoseDTO,
    SceneObjectDTO,
    WorldChangesDTO,
)
from world_manager.utils import expand_xacro

import xacro

_OWNER = 'tools_manager'
_MOVEIT_OBJECT_OFFSET_X = 0.001


class ToolsWorldClientAdapter:
    """Translate tool operations into typed Gazebo and MoveIt requests."""

    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        config: ToolsManagerConfigDTO,
        simulated: bool,
        station_model_name: str,
        timeout_sec: float = 15.0,
    ) -> None:
        """Create typed clients and retain tool-model configuration."""
        self._node = node
        self._config = config
        self._station_model_name = station_model_name
        self._moveit = MoveIt2WorldClient(
            node, callback_group, timeout_sec=timeout_sec
        )
        self._gazebo = (
            GazeboWorldClient(node, callback_group, timeout_sec=timeout_sec)
            if simulated
            else None
        )

    def activate(self) -> None:
        """Check required manager services before the tools are loaded."""
        self._moveit.list_objects()
        if self._gazebo is not None:
            self._gazebo.list_models(owner=_OWNER)

    def destroy(self) -> None:
        """Destroy the service clients."""
        self._moveit.destroy()
        if self._gazebo is not None:
            self._gazebo.destroy()

    def spawn_tool(
        self,
        tool: ToolInfoDto,
        link_name: str,
        rotation: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Add a tool to MoveIt and, when enabled, to the Gazebo world."""
        mesh_path = self._collision_mesh_path(tool)
        scene_object = SceneObjectDTO(
            object_id=tool.tool_sn,
            mesh_path=str(mesh_path),
            pose=self._moveit_pose(link_name, rotation),
            attach_link=link_name,
            touch_links=(link_name,),
        )
        self._moveit.apply(
            WorldChangesDTO(add_or_replace=(scene_object,))
        )

        if self._gazebo is None:
            return

        metadata = self._config.get_tool_metadata(tool.tool_sn)
        if metadata is None:
            raise WorldManagerError(
                f'No metadata defined for tool {tool.tool_sn}'
            )
        try:
            slot = self._config.get_tool_slot_data(tool.tool_sn)
        except ValueError as error:
            raise WorldManagerError(
                f'Tool {tool.tool_sn} has no configured rack slot'
            ) from error
        model_path = self._model_description_path(metadata.model)
        try:
            sdf = expand_xacro(
                model_path,
                {
                    'model_name': tool.tool_sn,
                    'station_model_name': self._station_model_name,
                    'tool_mount_link': 'tool_mount_tcp',
                    'tool_rack_link': slot.tool_attached_frame,
                    'tag_data': tool.to_tag_data().decode(),
                },
            )
        except (xacro.XacroException, OSError, ValueError) as error:
            raise WorldManagerError(
                f'Failed to expand model description for '
                f'{tool.tool_sn}: {error}'
            ) from error
        self._gazebo.spawn(
            [
                ModelSpecDTO(
                    model_id=tool.tool_sn,
                    sdf=sdf,
                    attach_link=link_name,
                    pose=PoseDTO(
                        frame_id=link_name,
                        qx=rotation[0] if rotation else 0.0,
                        qy=rotation[1] if rotation else 0.0,
                        qz=rotation[2] if rotation else 0.0,
                        qw=rotation[3] if rotation else 1.0,
                    ),
                )
            ],
            owner=_OWNER,
        )

    def delete_tool(self, model_id: str) -> None:
        """Remove a tool from MoveIt and from the owned Gazebo models."""
        try:
            self._moveit.apply(WorldChangesDTO(remove_ids=(model_id,)))
        except WorldManagerError as error:
            self._node.get_logger().error(
                f'Failed to remove {model_id} from the MoveIt scene: {error}'
            )
            raise
        if self._gazebo is not None:
            try:
                self._gazebo.delete(model_ids=(model_id,), owner=_OWNER)
            except WorldManagerError as error:
                self._node.get_logger().error(
                    f'Failed to remove {model_id} from Gazebo: {error}'
                )
                raise

    def attach_to_link(
        self,
        model_id: str,
        link_name: str,
        rotation: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Transfer a tool attachment in both world representations."""
        pose = PoseStamped()
        pose.header.frame_id = link_name
        pose.pose.position.x = _MOVEIT_OBJECT_OFFSET_X
        if rotation is None:
            pose.pose.orientation.w = 1.0
        else:
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = rotation
        self._moveit.attach(
            model_id,
            link_name,
            pose,
            touch_links=(link_name,),
        )
        if self._gazebo is not None:
            self._gazebo.set_attachment(model_id, link_name)

    def allow_collisions(
        self,
        model_id: str,
        allowed: bool,
        tool_mount_link: str,
        slot_link: str,
    ) -> bool:
        """Update the three pairwise tool/mount/rack collision allowances."""
        pairs = tuple(
            AllowedCollisionPairDTO(
                name_a=name_a,
                name_b=name_b,
                allowed=allowed,
            )
            for name_a, name_b in combinations(
                (model_id, tool_mount_link, slot_link), 2
            )
        )
        try:
            self._moveit.apply(WorldChangesDTO(acm_changes=pairs))
        except WorldManagerError as error:
            self._node.get_logger().error(
                'Failed to update collision allowances for '
                f'{model_id}: {error}'
            )
            return False
        return True

    @staticmethod
    def _moveit_pose(
        link_name: str,
        rotation: tuple[float, float, float, float] | None,
    ) -> PoseDTO:
        """Build the attached collision-object pose in the link frame."""
        quaternion = rotation or (0.0, 0.0, 0.0, 1.0)
        return PoseDTO(
            frame_id=link_name,
            x=_MOVEIT_OBJECT_OFFSET_X,
            qx=quaternion[0],
            qy=quaternion[1],
            qz=quaternion[2],
            qw=quaternion[3],
        )

    def _collision_mesh_path(self, tool: ToolInfoDto) -> Path:
        metadata = self._config.get_tool_metadata(tool.tool_sn)
        if metadata is None:
            raise WorldManagerError(
                f'No metadata defined for tool {tool.tool_sn}'
            )
        return self._tool_model_path(metadata.model, 'meshes/collision.stl')

    def _model_description_path(self, model_name: str) -> Path:
        return self._tool_model_path(model_name, 'model.sdf.xacro')

    @staticmethod
    def _tool_model_path(model_name: str, relative_path: str) -> Path:
        try:
            share = Path(get_package_share_directory('endtools'))
        except PackageNotFoundError as error:
            raise WorldManagerError(
                f"Failed to locate 'endtools' package share directory: {error}"
            ) from error
        path = share / 'model' / model_name / relative_path
        if not path.is_file():
            raise WorldManagerError(f'Tool model file not found: {path}')
        return path
