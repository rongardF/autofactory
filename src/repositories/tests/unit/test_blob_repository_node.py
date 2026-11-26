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
"""Lifecycle, handler and action tests for the ``blob_repository`` node."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from repositories import blob_repository as node_module
from repositories.action import BlobRepositoryFetchBlobs, BlobRepositoryUploadBlobs
from repositories.blob_repository import BlobRepository
from repositories.srv import (
    BlobRepositoryDeleteBlob,
    BlobRepositoryFetchBlob,
    BlobRepositoryGetBlobInfo,
    BlobRepositoryUploadBlob,
)

STATION_ID = '3f1c2a4e-8b7d-4c6a-9e2f-1a2b3c4d5e6f'
SUCCESS = TransitionCallbackReturn.SUCCESS
FAILURE = TransitionCallbackReturn.FAILURE
# rclpy reports the node-relative names as declared; they resolve to /blob_repository/<name>.
SERVICE_NAMES = {
    f'~/{name}' for name in ('upload_blob', 'fetch_blob', 'get_blob_info', 'delete_blob')
}


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def data_dir(tmp_path: Path) -> Path:
    data = tmp_path / 'data'
    data.mkdir()
    return data


@pytest.fixture
def node(tmp_path: Path, data_dir: Path):
    blob_node = BlobRepository()
    blob_node.set_parameters([
        Parameter('station_id', value=STATION_ID),
        Parameter('local_storage_path', value=str(tmp_path / 'store')),
        Parameter('allowed_path_roots', value=[str(data_dir)]),
        Parameter('cache_size_mb', value=10),
        Parameter('max_blob_size_mb', value=5),
    ])
    yield blob_node
    if blob_node.handle is not None:
        blob_node.destroy_node()


@pytest.fixture
def active(node: BlobRepository) -> BlobRepository:
    assert node.trigger_configure() == SUCCESS
    assert node.trigger_activate() == SUCCESS
    return node


def _service_names(blob_node: BlobRepository) -> set[str]:
    return {service.srv_name for service in blob_node.services} & SERVICE_NAMES


def _upload(
    blob_node: BlobRepository, source: Path, **fields
) -> BlobRepositoryUploadBlob.Response:
    request = BlobRepositoryUploadBlob.Request(source_path=str(source), **fields)
    return blob_node._on_upload_blob(request, BlobRepositoryUploadBlob.Response())


class FakeGoalHandle:
    """Records goal handle calls; can request cancellation after N feedbacks."""

    def __init__(self, request, cancel_after: int | None = None) -> None:
        self.request = request
        self.feedback: list = []
        self.status: str | None = None
        self._cancel_after = cancel_after

    @property
    def is_cancel_requested(self) -> bool:
        return self._cancel_after is not None and len(self.feedback) >= self._cancel_after

    def publish_feedback(self, feedback) -> None:
        self.feedback.append(feedback)

    def succeed(self) -> None:
        self.status = 'succeeded'

    def abort(self) -> None:
        self.status = 'aborted'

    def canceled(self) -> None:
        self.status = 'canceled'


# region: lifecycle


def test_configure_fails_with_invalid_parameters(node: BlobRepository) -> None:
    """Test that an invalid station_id fails configure with a logged error."""
    node.set_parameters([Parameter('station_id', value='')])

    assert node.trigger_configure() == FAILURE


def test_configure_fails_without_allowed_roots(tmp_path: Path) -> None:
    """Test that the undeclared-default allowlist fails configuration."""
    blob_node = BlobRepository()
    blob_node.set_parameters([
        Parameter('station_id', value=STATION_ID),
        Parameter('local_storage_path', value=str(tmp_path / 'store')),
    ])
    try:
        assert blob_node.trigger_configure() == FAILURE
    finally:
        blob_node.destroy_node()


def test_configure_fails_with_missing_allowed_root(node: BlobRepository, tmp_path: Path) -> None:
    """Test that a non-existent allowed root fails configuration."""
    node.set_parameters([Parameter('allowed_path_roots', value=[str(tmp_path / 'nope')])])

    assert node.trigger_configure() == FAILURE


def test_services_and_actions_exist_only_while_active(node: BlobRepository) -> None:
    """Test that activation creates and deactivation destroys the ROS surface."""
    assert node.trigger_configure() == SUCCESS
    assert _service_names(node) == set()

    assert node.trigger_activate() == SUCCESS
    assert _service_names(node) == SERVICE_NAMES
    assert len(node._blob_action_servers) == 2

    assert node.trigger_deactivate() == SUCCESS
    assert _service_names(node) == set()
    assert node._blob_action_servers == []
    assert node._cache_timer is None


@pytest.mark.parametrize('state', ['unconfigured', 'inactive', 'active'])
def test_shutdown_from_each_state_releases_resources_and_keeps_blobs(
    node: BlobRepository, data_dir: Path, tmp_path: Path, state: str
) -> None:
    """Test shutdown from each primary state; persisted blobs survive."""
    if state != 'unconfigured':
        assert node.trigger_configure() == SUCCESS
        assert node.trigger_activate() == SUCCESS
        (data_dir / 'a.bin').write_bytes(b'abc')
        assert _upload(node, data_dir / 'a.bin').success
        if state == 'inactive':
            assert node.trigger_deactivate() == SUCCESS

    assert node.trigger_shutdown() == SUCCESS

    assert _service_names(node) == set()
    assert node._manager is None and node._backend is None and node._config is None
    if state != 'unconfigured':
        assert len(list((tmp_path / 'store').iterdir())) == 2


def test_shutdown_after_cleanup_is_safe(node: BlobRepository) -> None:
    """Test that shutdown after deactivate and cleanup succeeds."""
    for transition in ('configure', 'activate', 'deactivate', 'cleanup'):
        assert getattr(node, f'trigger_{transition}')() == SUCCESS

    assert node.trigger_shutdown() == SUCCESS


def test_teardown_failure_is_logged_and_fails_transition(active: BlobRepository, mocker) -> None:
    """Test that a failing backend shutdown is logged and fails deactivation."""
    mocker.patch.object(active._backend, 'shutdown', side_effect=RuntimeError('stuck'))
    logger = MagicMock()
    mocker.patch.object(active, 'get_logger', return_value=logger)

    assert active.trigger_deactivate() == FAILURE
    assert 'backend shutdown failed: stuck' in logger.error.call_args.args[0]


def test_destroy_node_releases_resources_without_lifecycle_shutdown(
    active: BlobRepository, mocker
) -> None:
    """Test that destroying an active node shuts the backend down and drops state."""
    shutdown = mocker.spy(active._backend, 'shutdown')

    active.destroy_node()

    shutdown.assert_called_once()
    assert active._manager is None and active._blob_services == []


def test_main_releases_node_and_context_on_interrupt(mocker) -> None:
    """Test that main() destroys the node and shuts ROS down on process exit."""
    mocker.patch.object(node_module.rclpy, 'init')
    try_shutdown = mocker.patch.object(node_module.rclpy, 'try_shutdown')
    fake_node = MagicMock()
    mocker.patch.object(node_module, 'BlobRepository', return_value=fake_node)
    executor = MagicMock()
    executor.spin.side_effect = KeyboardInterrupt
    mocker.patch.object(node_module, 'MultiThreadedExecutor', return_value=executor)

    node_module.main()

    executor.shutdown.assert_called_once()
    fake_node.destroy_node.assert_called_once()
    try_shutdown.assert_called_once()


# endregion: lifecycle

# region: service handlers


def test_handlers_fail_cleanly_when_not_configured(node: BlobRepository) -> None:
    """Test that handlers return failure results instead of raising when unconfigured."""
    response = node._on_get_blob_info(
        BlobRepositoryGetBlobInfo.Request(blob_id=STATION_ID),
        BlobRepositoryGetBlobInfo.Response(),
    )

    assert response.success is False
    assert response.error_message == 'node is not configured'


def test_upload_fetch_info_delete_through_handlers(active: BlobRepository, data_dir: Path) -> None:
    """Test the service handlers end to end in local mode."""
    (data_dir / 'a.json').write_bytes(b'{"x": 1}')
    uploaded = _upload(active, data_dir / 'a.json', metadata='{"k": "v"}')
    assert uploaded.success and uploaded.media_type == 'application/json'

    fetched = active._on_fetch_blob(
        BlobRepositoryFetchBlob.Request(
            blob_id=uploaded.blob_id, destination_path=str(data_dir / 'out.json')
        ),
        BlobRepositoryFetchBlob.Response(),
    )
    assert fetched.success and (data_dir / 'out.json').read_bytes() == b'{"x": 1}'
    assert fetched.metadata == '{"k":"v"}'

    info = active._on_get_blob_info(
        BlobRepositoryGetBlobInfo.Request(blob_id=uploaded.blob_id),
        BlobRepositoryGetBlobInfo.Response(),
    )
    assert (info.success, info.size_bytes, info.sha256) == (True, 8, uploaded.sha256)

    deleted = active._on_delete_blob(
        BlobRepositoryDeleteBlob.Request(blob_id=uploaded.blob_id),
        BlobRepositoryDeleteBlob.Response(),
    )
    assert deleted.success


@pytest.mark.parametrize(
    ('fields', 'message'),
    [
        ({'metadata': '{bad json'}, 'metadata is not valid JSON'),
        ({'blob_id': 'not-a-uuid'}, 'UUID4'),
        ({'media_type': 'not a type'}, 'media_type'),
    ],
)
def test_malformed_upload_requests_return_logged_failures(
    active: BlobRepository, data_dir: Path, mocker, fields: dict, message: str
) -> None:
    """Test that malformed requests produce failure responses with safe defaults."""
    (data_dir / 'a.bin').write_bytes(b'abc')
    logger = MagicMock()
    mocker.patch.object(active, 'get_logger', return_value=logger)

    response = _upload(active, data_dir / 'a.bin', **fields)

    assert response.success is False and message in response.error_message
    assert (response.blob_id, response.size_bytes, response.sha256) == ('', 0, '')
    logger.error.assert_called_once()


def test_fetch_failure_resets_response_fields(active: BlobRepository) -> None:
    """Test that a failed fetch returns empty path and descriptor fields."""
    response = active._on_fetch_blob(
        BlobRepositoryFetchBlob.Request(blob_id=STATION_ID),
        BlobRepositoryFetchBlob.Response(),
    )

    assert response.success is False and 'blob not found' in response.error_message
    assert (response.local_path, response.blob_name, response.size_bytes) == ('', '', 0)


def test_cache_timer_failures_are_logged_not_raised(active: BlobRepository, mocker) -> None:
    """Test that the periodic cache check never lets an exception escape."""
    mocker.patch.object(active._manager, 'enforce_cache_limit', side_effect=OSError('io'))
    logger = MagicMock()
    mocker.patch.object(active, 'get_logger', return_value=logger)

    active._on_cache_timer()

    assert 'cache size check failed' in logger.error.call_args.args[0]


# endregion: service handlers

# region: actions


def test_upload_action_reports_ordered_feedback(active: BlobRepository, data_dir: Path) -> None:
    """Test that batch upload publishes one ordered feedback per item and succeeds."""
    paths = []
    for index in range(3):
        (data_dir / f'{index}.bin').write_bytes(bytes([index]))
        paths.append(str(data_dir / f'{index}.bin'))
    goal = FakeGoalHandle(BlobRepositoryUploadBlobs.Goal(source_paths=paths))

    result = active._execute_upload_blobs(goal)

    assert result.success and goal.status == 'succeeded'
    assert [(f.completed, f.total, f.source_path) for f in goal.feedback] == [
        (1, 3, paths[0]), (2, 3, paths[1]), (3, 3, paths[2])
    ]
    assert [f.blob_id for f in goal.feedback] == list(result.blob_ids)


@pytest.mark.parametrize(
    'goal_fields',
    [{'source_paths': []}, {'source_paths': ['/a', '/b'], 'blob_names': ['only-one']}],
)
def test_upload_action_validates_before_side_effects(
    active: BlobRepository, tmp_path: Path, goal_fields: dict
) -> None:
    """Test that empty batches and mismatched arrays abort before any work."""
    goal = FakeGoalHandle(BlobRepositoryUploadBlobs.Goal(**goal_fields))

    result = active._execute_upload_blobs(goal)

    assert result.success is False and goal.status == 'aborted'
    assert list(result.blob_ids) == [] and goal.feedback == []
    assert list((tmp_path / 'store').iterdir()) == []


def test_upload_action_cancel_rolls_back(
    active: BlobRepository, data_dir: Path, tmp_path: Path
) -> None:
    """Test that canceling between items rolls back and reports cancellation."""
    paths = []
    for index in range(3):
        (data_dir / f'{index}.bin').write_bytes(b'x')
        paths.append(str(data_dir / f'{index}.bin'))
    goal = FakeGoalHandle(BlobRepositoryUploadBlobs.Goal(source_paths=paths), cancel_after=1)

    result = active._execute_upload_blobs(goal)

    assert result.success is False and goal.status == 'canceled'
    assert 'canceled after 1 of 3' in result.error_message
    assert list((tmp_path / 'store').iterdir()) == []


def test_fetch_action_is_fail_fast(active: BlobRepository, data_dir: Path) -> None:
    """Test that batch fetch stops at the first unknown blob without returning paths."""
    (data_dir / 'a.bin').write_bytes(b'abc')
    blob_id = _upload(active, data_dir / 'a.bin').blob_id
    goal = FakeGoalHandle(BlobRepositoryFetchBlobs.Goal(blob_ids=[blob_id, STATION_ID]))

    result = active._execute_fetch_blobs(goal)

    assert result.success is False and goal.status == 'aborted'
    assert list(result.local_paths) == []
    assert [f.blob_id for f in goal.feedback] == [blob_id]


def test_fetch_action_overwrites_destinations(active: BlobRepository, data_dir: Path) -> None:
    """Test that batch fetch replaces existing destination files on success."""
    (data_dir / 'a.bin').write_bytes(b'new')
    blob_id = _upload(active, data_dir / 'a.bin').blob_id
    target = data_dir / 'target.bin'
    target.write_bytes(b'old content')
    goal = FakeGoalHandle(
        BlobRepositoryFetchBlobs.Goal(blob_ids=[blob_id], destination_paths=[str(target)])
    )

    result = active._execute_fetch_blobs(goal)

    assert result.success and list(result.local_paths) == [str(target)]
    assert target.read_bytes() == b'new'


# endregion: actions
