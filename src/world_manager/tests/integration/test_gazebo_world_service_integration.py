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
"""Headless-Gazebo integration test of GazeboWorldService (marker ``gazebo``; skipped without ``gz``)."""

from os import environ, getpid, setsid, killpg, getpgid
from pathlib import Path
import sys
from shutil import which
from subprocess import DEVNULL, Popen
from time import monotonic, sleep
from unittest.mock import MagicMock

from pytest import fixture, mark, raises
from rclpy.impl.rcutils_logger import RcutilsLogger

from world_manager.enums import JointStateEnum
from world_manager.exceptions import GzRequestError, ModelValidationError
from world_manager.models import GazeboConfigDTO, ModelSpecDTO, PoseDTO
from world_manager.services.detachable_joint_controller import DetachableJointController
from world_manager.services.gazebo_world_service import GazeboWorldService
from world_manager.services.model_registry import ModelRegistry
from world_manager.services.pose_resolver import PoseResolver
from world_manager.services.real_gz_transport_adapter import RealGzTransportAdapter
from world_manager.services.sdf_inspector import SdfInspector

ASSETS = Path(__file__).parent / 'assets'
WORLD = 'spike_world'

pytestmark = [mark.gazebo, mark.skipif(which('gz') is None, reason='gz is not installed')]


@fixture(scope='module')
def gz_world():
    """Run a headless Gazebo server on a private partition."""
    environ['GZ_PARTITION'] = f'world_manager_test_{getpid()}'
    environ.setdefault('GZ_IP', '127.0.0.1')
    server = Popen(['gz', 'sim', '-s', '-r', str(ASSETS / 'world.sdf')],
                   stdout=DEVNULL, stderr=DEVNULL, preexec_fn=setsid)
    try:
        yield server
    finally:
        killpg(getpgid(server.pid), 15)
        server.wait(timeout=10)


@fixture(scope='module')
def service(gz_world):
    config = GazeboConfigDTO(world_name=WORLD, confirm_timeout_sec=5.0, operation_timeout_sec=20.0)
    logger = MagicMock(spec=RcutilsLogger)
    adapter = RealGzTransportAdapter(logger)
    registry = ModelRegistry()
    controller = DetachableJointController(adapter, registry, config.republish_period_sec, logger)
    svc = GazeboWorldService(
        adapter, registry, controller, SdfInspector(),
        PoseResolver(lambda *_: PoseDTO(), config.world_frame, 1.0), config, logger)
    deadline = monotonic() + 30.0
    while monotonic() < deadline:
        try:
            svc.activate()
            break
        except Exception:
            sleep(0.5)
    else:
        raise RuntimeError('Gazebo did not come up')
    return svc


def _spec(file: str, model_id: str, attach_link: str = '') -> ModelSpecDTO:
    return ModelSpecDTO(
        model_id=model_id, sdf=(ASSETS / file).read_text(), attach_link=attach_link,
        pose=PoseDTO(x=0.5, z=1.0))


def _states(service: GazeboWorldService, model_id: str) -> dict[str, JointStateEnum]:
    model = next(m for m in service.list_models() if m.model_id == model_id)
    return {j.link_name: j.state for j in model.joints}


def test_spawn_transfer_set_pose_delete(service):
    """Spawn a two-joint tool, move it between links, set its pose and delete it, all in real Gazebo."""
    service.spawn([_spec('tool_two_joints.sdf', 'tool_1', attach_link='tool_mount_tcp')], 'it')
    assert _states(service, 'tool_1') == {
        'tool_mount_tcp': JointStateEnum.ATTACHED, 'rack_link': JointStateEnum.DETACHED}

    service.set_attachment('tool_1', 'rack_link')
    assert _states(service, 'tool_1') == {
        'tool_mount_tcp': JointStateEnum.DETACHED, 'rack_link': JointStateEnum.ATTACHED}

    service.set_attachment('tool_1', '')
    service.set_pose('tool_1', PoseDTO(x=1.0, y=1.0, z=2.0))

    result, error = service.delete(owner='it')
    assert (result.deleted_ids, error) == (('tool_1',), '')
    assert service.reconcile() == ['station']


def test_failed_spawn_rolls_back_in_real_gazebo(service):
    """A refused second model rolls back the first and leaves the world as it was."""
    carrier = _spec('carrier_one_joint.sdf', 'carrier_1')
    broken = ModelSpecDTO(
        model_id='broken',
        sdf='<sdf version="1.9"><model name="broken"><link name="l"/><link name="l"/></model></sdf>')
    with raises(GzRequestError, match='broken'):
        service.spawn([carrier, broken], 'it')
    assert service.reconcile() == ['station']
    assert service.list_models() == []


def test_existing_unmanaged_model_is_not_adopted(service):
    """The pre-existing 'station' model is reported as unmanaged when its id is spawned."""
    spec = ModelSpecDTO(model_id='station', sdf='<sdf version="1.9"><model name="station"/></sdf>')
    with raises(ModelValidationError, match='not managed'):
        service.spawn([spec], 'it')


_FLOOD_PUBLISHER = '''
import time
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
node = Node()
publisher = node.advertise('/world_manager_test/flood', StringMsg)
while True:
    publisher.publish(StringMsg(data='attached'))
    time.sleep(0.001)
'''


def test_requests_answer_while_subscribed_messages_arrive(service):
    """Messages on a subscribed topic arriving during a request must not stall the reply.

    gz-transport's Python binding holds the GIL during a blocking request while subscription callbacks
    need it on the reception thread, so such a request used to time out although Gazebo had answered.
    """
    adapter = RealGzTransportAdapter(MagicMock(spec=RcutilsLogger))
    received: list[str] = []
    adapter.subscribe_state('/world_manager_test/flood', received.append)
    publisher = Popen([sys.executable, '-c', _FLOOD_PUBLISHER], stdout=DEVNULL, stderr=DEVNULL)
    try:
        deadline = monotonic() + 10.0
        while not received and monotonic() < deadline:
            sleep(0.1)
        assert received, 'flood publisher not received'
        for _ in range(5):
            start = monotonic()
            assert adapter.list_models(WORLD, 2.0) is not None
            assert monotonic() - start < 1.0
    finally:
        publisher.kill()
        publisher.wait()
        adapter.close()
