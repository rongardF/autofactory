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
"""Tests for the tools-manager adapter to shared world clients."""

from pathlib import Path
from unittest.mock import Mock

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolSlotDTO
from tools_manager.services import world_client_adapter as adapter_module
from tools_manager.services.world_client_adapter import ToolsWorldClientAdapter
from tools_manager.services.world_client_adapter import ToolsWorldClientAdapter


class _FakeMoveItClient:
    def __init__(self, *args, **kwargs) -> None:
        self.changes = []
        self.attachments = []
        self.listed = False

    def list_objects(self) -> list:
        self.listed = True
        return []

    def apply(self, changes) -> None:
        self.changes.append(changes)

    def attach(self, *args, **kwargs) -> None:
        self.attachments.append((args, kwargs))

    def destroy(self) -> None:
        pass


class _FakeGazeboClient:
    def __init__(self, *args, **kwargs) -> None:
        self.spawned = []
        self.deleted = []
        self.attachments = []
        self.listed = False

    def list_models(self, owner='') -> list:
        self.listed = True
        return []

    def spawn(self, models, owner, cancel=None) -> list[str]:
        self.spawned.append((models, owner))
        return [model.model_id for model in models]

    def delete(self, model_ids=(), owner='', cancel=None):
        self.deleted.append((model_ids, owner))

    def set_attachment(self, model_id, link_name, cancel=None) -> None:
        self.attachments.append((model_id, link_name))

    def destroy(self) -> None:
        pass


def _adapter(monkeypatch, model_share: Path):
    monkeypatch.setattr(
        adapter_module,
        'MoveIt2WorldClient',
        _FakeMoveItClient,
    )
    monkeypatch.setattr(adapter_module, 'GazeboWorldClient', _FakeGazeboClient)
    monkeypatch.setattr(
        adapter_module,
        'get_package_share_directory',
        lambda _: str(model_share),
    )
    xacro_expander = Mock(return_value='sdf')
    monkeypatch.setattr(adapter_module, 'expand_xacro', xacro_expander)
    return ToolsWorldClientAdapter(
        node=Mock(),
        callback_group=Mock(),
        simulated=True,
        station_model_name='cell_station',
    )


def test_spawn_builds_moveit_and_gazebo_requests(
    monkeypatch,
    tmp_path,
) -> None:
    """Tool mesh, SDF mappings, owner and mount pose reach both clients."""
    model_dir = tmp_path / 'model' / 'tool_model'
    (model_dir / 'meshes').mkdir(parents=True)
    (model_dir / 'meshes' / 'collision.stl').touch()
    (model_dir / 'model.sdf.xacro').touch()
    adapter = _adapter(monkeypatch, tmp_path)
    tool = ToolInfoDto(index=-1, tool_sn='SN-01')
    rotation = (0.0, 0.0, 1.0, 0.0)

    adapter.activate()
    adapter.spawn_tool(
        tool,
        model_name='tool_model',
        home_slot=ToolSlotDTO(index=2, tool_sn='SN-01', node_name='node'),
        link_name='tool_mount_tcp',
        rotation=rotation,
    )

    scene_object = adapter._moveit.changes[0].add_or_replace[0]
    assert scene_object.object_id == 'SN-01'
    assert scene_object.attach_link == 'tool_mount_tcp'
    assert scene_object.touch_links == ('tool_mount_tcp',)
    assert scene_object.pose.x == 0.001
    assert scene_object.pose.qz == 1.0
    assert scene_object.pose.qw == 0.0

    models, owner = adapter._gazebo.spawned[0]
    assert owner == 'tools_manager'
    assert models[0].model_id == 'SN-01'
    assert models[0].attach_link == 'tool_mount_tcp'
    assert models[0].pose.frame_id == 'tool_mount_tcp'
    assert models[0].pose.qz == 1.0
    mappings = adapter_module.expand_xacro.call_args.args[1]
    assert mappings['station_model_name'] == 'cell_station'
    assert mappings['tool_rack_link'] == 'slot2_attached_link'
    assert mappings['tag_data'] == tool.to_tag_data().decode()
    assert adapter._moveit.listed
    assert adapter._gazebo.listed


def test_attachment_collision_and_delete_use_shared_clients(
    monkeypatch,
    tmp_path,
) -> None:
    """Transfers, ACM pairs and cleanup are sent through the shared clients."""
    model_dir = tmp_path / 'model' / 'tool_model'
    (model_dir / 'meshes').mkdir(parents=True)
    (model_dir / 'meshes' / 'collision.stl').touch()
    (model_dir / 'model.sdf.xacro').touch()
    adapter = _adapter(monkeypatch, tmp_path)

    adapter.attach_to_link('SN-01', 'slot2_attached_link')
    assert adapter._moveit.attachments[0][0][:2] == (
        'SN-01',
        'slot2_attached_link',
    )
    assert adapter._moveit.attachments[0][0][2].header.frame_id == (
        'slot2_attached_link'
    )
    assert adapter._gazebo.attachments == [
        ('SN-01', 'slot2_attached_link'),
    ]

    assert adapter.allow_collisions(
        'SN-01',
        True,
        'tool_mount_tcp',
        'tool_rack_link',
    )
    pairs = adapter._moveit.changes[0].acm_changes
    assert {
        frozenset((pair.name_a, pair.name_b))
        for pair in pairs
    } == {
        frozenset(('SN-01', 'tool_mount_tcp')),
        frozenset(('SN-01', 'tool_rack_link')),
        frozenset(('tool_mount_tcp', 'tool_rack_link')),
    }

    adapter.delete_tool('SN-01')
    assert adapter._moveit.changes[1].remove_ids == ('SN-01',)
    assert adapter._gazebo.deleted == [(('SN-01',), 'tools_manager')]
