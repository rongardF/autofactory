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
"""Shared fixtures: a scripted fake of the gz-transport adapter and SDF builders."""

from dataclasses import dataclass, field
from threading import Event
from typing import Callable
from unittest.mock import MagicMock

from pytest import fixture
from rclpy.impl.rcutils_logger import RcutilsLogger

from world_manager.models import GazeboConfigDTO, ModelSpecDTO, PoseDTO
from world_manager.services.detachable_joint_controller import DetachableJointController
from world_manager.services.gazebo_world_service import GazeboWorldService
from world_manager.services.gz_transport_adapter import GzTransportAdapter
from world_manager.services.model_registry import ModelRegistry
from world_manager.services.pose_resolver import PoseResolver
from world_manager.services.sdf_inspector import SdfInspector

WORLD = 'test_world'


def make_sdf(model: str, joints: list[tuple] = (), body: str = '') -> str:
    """Build an SDF; each joint is (child_model, child_link) or (child_model, child_link, attach, detach, state)."""
    plugins = ''
    for joint in joints:
        child_model, child_link, *topics = joint
        explicit = ''
        if topics:
            attach, detach, state = topics
            explicit = (f'<attach_topic>{attach}</attach_topic><detach_topic>{detach}</detach_topic>'
                        f'<output_topic>{state}</output_topic>')
        plugins += (
            '<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">'
            f'<parent_link>origin</parent_link><child_model>{child_model}</child_model>'
            f'<child_link>{child_link}</child_link>{explicit}</plugin>')
    return f'<?xml version="1.0"?><sdf version="1.9"><model name="{model}">{body}{plugins}</model></sdf>'


def tool_sdf(model: str, links: tuple[str, ...] = ('mount', 'rack')) -> str:
    """Two-joint tool with unique explicit topics, like the endtools SDFs."""
    return make_sdf(model, [
        ('station', link, f'/{link}/{model}/attach', f'/{link}/{model}/detach', f'/{link}/{model}/state')
        for link in links
    ])


def spec(
    model: str, sdf: str | None = None, attach_link: str = '', frame_id: str = '', delete_with_child: bool = False,
) -> ModelSpecDTO:
    return ModelSpecDTO(
        model_id=model, sdf=sdf if sdf is not None else tool_sdf(model), attach_link=attach_link,
        delete_with_child=delete_with_child, pose=PoseDTO(frame_id=frame_id))


class FakeGzAdapter(GzTransportAdapter):
    """Scripted gz: records calls and behaves like the DetachableJoint plugin (state events on change)."""

    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.world: set[str] = {'station'}
        self.fail_create: set[str] = set()
        self.late_create: set[str] = set()
        self.fail_remove: set[str] = set()
        self.fail_set_pose = False
        self.scene_answers = True
        self.subscribe_ok = True
        self.send_initial_event = True
        self.swallow_publishes = 0
        self.ignore_topics: set[str] = set()
        self.subscribers: dict[str, Callable[[str], None]] = {}
        self._joints: dict[str, dict] = {}
        self._inspector = SdfInspector()
        self.on_create: Callable[[str], None] | None = None

    def names(self, verb: str) -> list[str]:
        return [call[1] for call in self.calls if call[0] == verb]

    def emit(self, topic: str, data: str) -> None:
        if topic in self.subscribers:
            self.subscribers[topic](data)

    def create_model(self, world, model_id, sdf, pose, timeout_sec):
        self.calls.append(('create', model_id))
        if model_id in self.late_create:
            self.world.add(model_id)  # created, but the reply never arrives in time
            return False
        if model_id in self.fail_create or model_id in self.world:
            return False
        self.world.add(model_id)
        for joint in self._inspector.inspect(sdf).joints:
            self._joints[joint.attach_topic] = {'state_topic': joint.state_topic, 'attached': True, 'verb': 'attached'}
            self._joints[joint.detach_topic] = {'state_topic': joint.state_topic, 'attached': True, 'verb': 'detached'}
            self._joints[joint.attach_topic]['peer'] = joint.detach_topic
            self._joints[joint.detach_topic]['peer'] = joint.attach_topic
            if self.send_initial_event:
                self.emit(joint.state_topic, 'attached')
        if self.on_create:
            self.on_create(model_id)
        return True

    def remove_model(self, world, model_id, timeout_sec):
        self.calls.append(('remove', model_id))
        if model_id in self.fail_remove or model_id not in self.world:
            return False
        self.world.discard(model_id)
        return True

    def set_model_pose(self, world, model_id, pose, timeout_sec):
        self.calls.append(('set_pose', model_id, pose))
        return not self.fail_set_pose and model_id in self.world

    def list_models(self, world, timeout_sec):
        self.calls.append(('scene_info',))
        return sorted(self.world) if self.scene_answers else None

    def subscribe_state(self, topic, callback):
        self.calls.append(('subscribe', topic))
        if not self.subscribe_ok:
            return False
        self.subscribers[topic] = callback
        return True

    def unsubscribe(self, topic):
        self.calls.append(('unsubscribe', topic))
        self.subscribers.pop(topic, None)

    def publish_empty(self, topic):
        self.calls.append(('publish', topic))
        if self.swallow_publishes > 0:
            self.swallow_publishes -= 1
            return
        joint = self._joints.get(topic)
        if joint is None or topic in self.ignore_topics:
            return
        target = joint['verb'] == 'attached'
        peer = self._joints[joint['peer']]
        if joint['attached'] == target:
            return  # no state is emitted for a no-op request
        joint['attached'] = peer['attached'] = target
        self.emit(joint['state_topic'], joint['verb'])


@dataclass
class Stack:
    adapter: FakeGzAdapter
    registry: ModelRegistry
    controller: DetachableJointController
    service: GazeboWorldService
    config: GazeboConfigDTO
    dirty: Event
    tf: dict = field(default_factory=dict)


@fixture
def config() -> GazeboConfigDTO:
    return GazeboConfigDTO(
        world_name=WORLD, confirm_timeout_sec=0.4, republish_period_sec=0.02,
        operation_timeout_sec=5.0, rollback_timeout_sec=0.2, service_timeout_sec=1.0)


@fixture
def logger() -> RcutilsLogger:
    return MagicMock(spec=RcutilsLogger)


@fixture
def stack(config, logger) -> Stack:
    adapter = FakeGzAdapter()
    dirty = Event()
    registry = ModelRegistry(on_change=dirty.set)
    controller = DetachableJointController(adapter, registry, config.republish_period_sec, logger)
    tf: dict[str, PoseDTO] = {}

    def lookup(target: str, source: str, timeout: float) -> PoseDTO:
        return tf[source]

    service = GazeboWorldService(
        adapter, registry, controller, SdfInspector(), PoseResolver(lookup, config.world_frame, 1.0), config, logger)
    return Stack(adapter, registry, controller, service, config, dirty, tf)
