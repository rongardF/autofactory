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
# 3. Neither the name of the copyright holder nor its
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
"""Unit tests for typed client request conversion and bounded service calls."""

from concurrent.futures import Future
from threading import Event

from geometry_msgs.msg import PoseStamped
from pytest import raises
from shape_msgs.msg import SolidPrimitive

from world_manager.client import GazeboWorldClient, MoveIt2WorldClient
from world_manager.enums import JointStateEnum
from world_manager.exceptions import WorldManagerError
from world_manager.models import (
    AllowedCollisionPairDTO,
    ModelSpecDTO,
    PoseDTO,
    SceneObjectDTO,
    SolidPrimitiveDTO,
    WorldChangesDTO,
)
from world_manager.srv import (
    ApplyWorldChanges,
    AttachObject,
    DeleteModels,
    ListModels,
    ListObjects,
    SetModelAttachment,
    SetModelPose,
    SpawnModels,
)
from world_manager.utils import call_service


class FakeClient:
    def __init__(self, service_name: str, response=None) -> None:
        self.srv_name = service_name
        self.response = response
        self.requests = []
        self.available = True
        self.pending = Future()
        self.removed = False

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        if self.response is not None:
            self.pending = Future()
            self.pending.set_result(self.response)
        return self.pending

    def remove_pending_request(self, future) -> None:
        self.removed = True


class FakeNode:
    def __init__(self, responses) -> None:
        self.responses = iter(responses)
        self.clients = []

    def create_client(self, service_type, service_name, callback_group=None):
        client = FakeClient(service_name, next(self.responses))
        client.service_type = service_type
        self.clients.append(client)
        return client

    def destroy_client(self, client) -> None:
        self.clients.remove(client)


def test_call_service_reports_unavailable_server() -> None:
    """A missing manager fails after a bounded wait."""
    client = FakeClient('/unavailable')
    client.available = False
    with raises(WorldManagerError, match='not available'):
        call_service(client, object(), 0.001)
    assert client.requests == []


def test_call_service_cancellation_before_send() -> None:
    """A pre-cancelled call does not send a request."""
    client = FakeClient('/cancelled')
    cancel = Event()
    cancel.set()
    with raises(WorldManagerError, match='cancelled'):
        call_service(client, object(), 1.0, cancel)
    assert client.requests == []


def test_call_service_timeout_abandons_remote_future() -> None:
    """A response timeout cancels local waiting and marks the outcome unknown."""
    client = FakeClient('/slow')
    with raises(WorldManagerError, match='outcome may be unknown'):
        call_service(client, object(), 0.02)
    assert client.removed is True
    assert client.pending.cancelled()


def test_gazebo_client_serializes_spawn_and_maps_joint_state() -> None:
    """Gazebo client converts ModelSpecDTO and response messages into typed values."""
    spawned = SpawnModels.Response(success=True, spawned_ids=['tool'])
    listed = ListModels.Response(success=True)
    model = listed.models
    from world_manager.msg import JointInfo, ModelInfo
    joint = JointInfo(link_name='mount', child_model='station', state='attached',
                      state_confirmed=True, controllable=True)
    model.append(ModelInfo(model_id='tool', owner='test', joints=[joint]))
    node = FakeNode([
        spawned,
        DeleteModels.Response(success=True, deleted_ids=['tool']),
        SetModelPose.Response(success=True),
        SetModelAttachment.Response(success=True),
        listed,
    ])
    client = GazeboWorldClient(node, callback_group=None)

    ids = client.spawn(
        [ModelSpecDTO(model_id='tool', sdf='<sdf/>', pose=PoseDTO(frame_id='world', x=1.0))],
        'test',
    )
    request = node.clients[0].requests[0]
    assert ids == ['tool']
    assert request.owner == 'test'
    assert request.models[0].model_id == 'tool'
    assert request.models[0].pose.header.frame_id == 'world'
    assert request.models[0].pose.pose.position.x == 1.0
    result = client.list_models()
    assert result[0].joints[0].state == JointStateEnum.ATTACHED
    deleted = client.delete(['tool'])
    assert deleted.deleted_ids == ('tool',)


def test_gazebo_client_converts_failed_response_to_world_manager_error() -> None:
    """An unsuccessful ROS response raises the public client exception."""
    node = FakeNode([
        SpawnModels.Response(success=True),
        DeleteModels.Response(success=True),
        SetModelPose.Response(success=False, error_message='unknown model'),
        SetModelAttachment.Response(success=True),
        ListModels.Response(success=True),
    ])
    client = GazeboWorldClient(node, callback_group=None)
    with raises(WorldManagerError, match='unknown model'):
        client.set_pose('missing', PoseStamped())


def test_gazebo_client_sets_attachment_request_fields() -> None:
    """SetModelAttachment carries both the id and requested link."""
    node = FakeNode([
        SpawnModels.Response(success=True),
        DeleteModels.Response(success=True),
        SetModelPose.Response(success=True),
        SetModelAttachment.Response(success=True),
        ListModels.Response(success=True),
    ])
    client = GazeboWorldClient(node, callback_group=None)
    client.set_attachment('tool', 'mount')
    request = node.clients[3].requests[0]
    assert (request.model_id, request.link_name) == ('tool', 'mount')


def test_moveit_client_serializes_world_changes_and_maps_objects() -> None:
    """MoveIt client converts primitive geometry and ACM changes to the ROS request."""
    node = FakeNode([
        ApplyWorldChanges.Response(success=True),
        AttachObject.Response(success=True),
        ListObjects.Response(success=True),
    ])
    client = MoveIt2WorldClient(node, callback_group=None)
    object_dto = SceneObjectDTO(
        object_id='box',
        primitives=(SolidPrimitiveDTO(type=SolidPrimitive.BOX, dimensions=(1.0, 2.0, 3.0)),),
        primitive_poses=(PoseDTO(qw=1.0),),
        pose=PoseDTO(frame_id='world', z=0.5),
    )
    changes = WorldChangesDTO(
        add_or_replace=(object_dto,),
        acm_changes=(AllowedCollisionPairDTO(name_a='box', name_b='finger', allowed=True),),
    )
    client.apply(changes)
    request = node.clients[0].requests[0]
    assert request.add_or_replace[0].object_id == 'box'
    assert request.add_or_replace[0].primitives[0].type == SolidPrimitive.BOX
    assert tuple(request.add_or_replace[0].primitives[0].dimensions) == (1.0, 2.0, 3.0)
    assert request.acm_changes[0].allowed is True

    from world_manager.msg import ObjectInfo
    response = node.clients[2].response
    response.objects.append(ObjectInfo(object_id='box', attach_link='tool0'))
    assert client.list_objects()[0].attach_link == 'tool0'


def test_moveit_client_attach_serializes_pose_and_touch_links() -> None:
    """Attach sends the requested link, pose and touch links."""
    node = FakeNode([
        ApplyWorldChanges.Response(success=True),
        AttachObject.Response(success=True),
        ListObjects.Response(success=True),
    ])
    client = MoveIt2WorldClient(node, callback_group=None)
    pose = PoseStamped()
    pose.header.frame_id = 'tool0'
    pose.pose.position.x = 0.1
    client.attach('tool', 'tool0', pose, ('finger', 'tool0'))
    request = node.clients[1].requests[0]
    assert request.object_id == 'tool'
    assert request.link_name == 'tool0'
    assert request.pose.pose.position.x == 0.1
    assert request.touch_links == ['finger', 'tool0']


def test_moveit_client_converts_failed_response_to_world_manager_error() -> None:
    """An unsuccessful planning-scene response is raised as a client error."""
    node = FakeNode([
        ApplyWorldChanges.Response(success=False, error_message='MoveIt refused'),
        AttachObject.Response(success=True),
        ListObjects.Response(success=True),
    ])
    client = MoveIt2WorldClient(node, callback_group=None)
    with raises(WorldManagerError, match='MoveIt refused'):
        client.apply(WorldChangesDTO())
