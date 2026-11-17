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
#    contributors may be used to endorse products derived from this software
#    without specific prior written permission.
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
"""Unit tests for the workplace planning-scene client adapter."""

from pathlib import Path
from threading import Event
from unittest.mock import Mock

from geometry_msgs.msg import Pose

from shape_msgs.msg import SolidPrimitive

from station_workplace.models.common_node_config_dto import (
    SafetyObjectConfigDTO,
)

from station_workplace.services import planning_scene_adapter as adapter_module
from station_workplace.services.planning_scene_adapter import (
    PlanningSceneAdapter,
)

from world_manager.models import WorldChangesDTO


class _FakeClient:
    def __init__(self, *args, **kwargs) -> None:
        self.changes: list[WorldChangesDTO] = []
        self.destroyed = False

    def list_objects(self, cancel=None) -> list:
        return []

    def apply(self, changes: WorldChangesDTO, cancel=None) -> None:
        self.changes.append(changes)

    def destroy(self) -> None:
        self.destroyed = True


def _adapter(
    monkeypatch,
) -> tuple[PlanningSceneAdapter, _FakeClient, Mock]:
    node = Mock()
    node.get_logger.return_value = Mock()
    monkeypatch.setattr(adapter_module, 'MoveIt2WorldClient', _FakeClient)
    instance = PlanningSceneAdapter(
        node=node,
        attach_link='carrier',
        collision_allowed_link='conveyor_origin',
        safety_size=SafetyObjectConfigDTO(x=0.25, y=0.4, z=0.3),
        timeout_sec=2.0,
        callback_group=Mock(),
    )
    instance.activate()
    client = instance._client
    assert isinstance(client, _FakeClient)
    return instance, client, node


def test_safety_box_is_attached_with_expected_primitive(monkeypatch) -> None:
    """The adapter builds the box dimensions and offset expected by MoveIt."""
    adapter, client, _ = _adapter(monkeypatch)

    adapter.attach_safety_object(Event())

    changes = client.changes[0]
    assert len(changes.add_or_replace) == 1
    safety = changes.add_or_replace[0]
    assert safety.object_id == 'safety_object_model'
    assert safety.attach_link == 'carrier'
    assert safety.touch_links == ('conveyor_origin',)
    assert safety.primitives[0].type == SolidPrimitive.BOX
    assert safety.primitives[0].dimensions == (0.25, 0.4, 0.3)
    assert safety.primitive_poses[0].x == -0.125
    assert safety.primitive_poses[0].z == 0.15


def test_product_and_carrier_are_added_in_one_scene_diff(monkeypatch) -> None:
    """Both meshes and the ACM pair are included in the replacement diff."""
    adapter, client, _ = _adapter(monkeypatch)
    pose = Pose()
    pose.position.x = 0.12
    pose.orientation.w = 1.0

    adapter.attach_product_and_carrier(
        Path('/models/product.stl'),
        Path('/models/carrier.stl'),
        pose,
        Event(),
    )

    changes = client.changes[0]
    assert [item.object_id for item in changes.add_or_replace] == [
        'carrier_object_model',
        'product_object_model',
    ]
    assert changes.add_or_replace[0].mesh_path == '/models/carrier.stl'
    assert changes.add_or_replace[0].touch_links == ('conveyor_origin',)
    assert changes.add_or_replace[1].pose.frame_id == 'carrier'
    assert changes.add_or_replace[1].pose.x == 0.12
    assert changes.remove_ids == ('safety_object_model',)
    assert len(changes.acm_changes) == 1
    pair = changes.acm_changes[0]
    assert (pair.name_a, pair.name_b, pair.allowed) == (
        'product_object_model',
        'carrier_object_model',
        True,
    )


def test_removal_deduplicates_ids_and_deactivation_destroys_client(
    monkeypatch,
) -> None:
    """Removal is deduplicated and deactivation releases the service client."""
    adapter, client, _ = _adapter(monkeypatch)

    adapter.remove_objects(
        [
            'product_object_model',
            'carrier_object_model',
            'product_object_model',
        ],
        Event(),
    )
    adapter.deactivate()

    assert client.changes[0].remove_ids == (
        'product_object_model',
        'carrier_object_model',
    )
    assert client.destroyed
