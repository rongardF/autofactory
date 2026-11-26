#!/usr/bin/env python3
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
"""Blob (arbitrary file) upload/fetch lifecycle node.

``blob_repository`` stores and retrieves arbitrary files as opaque byte streams.
Clients upload by filesystem path and receive a UUID4 blob id; they fetch by id
and receive a local path. In local mode the local directory is the source of
truth; in cloud mode a station-scoped REST service is, and the local directory
is a size-limited cache. The node owns the ROS surface and parameter handling
and delegates storage to :class:`BlobManager`. Configuration is frozen on
``configure``; services and actions exist only while ``active``.
"""

from pathlib import Path

from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.service import Service
from rclpy.timer import Timer

from repositories.action import BlobRepositoryFetchBlobs, BlobRepositoryUploadBlobs
from repositories.exceptions.blob_repository import (
    BlobOperationCanceledError,
    BlobRepositoryError,
)
from repositories.interfaces.blob_repository.blob_backend import BlobBackend
from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobFetchResultDTO,
    BlobUploadItemDTO,
)
from repositories.models.blob_repository.blob_repository_config_dto import (
    BlobRepositoryConfigDTO,
)
from repositories.services.blob_repository.blob_manager import BlobManager
from repositories.services.blob_repository.local_blob_backend import LocalBlobBackend
from repositories.services.blob_repository.local_blob_store import LocalBlobStore
from repositories.services.blob_repository.path_policy import PathPolicy
from repositories.services.blob_repository.remote_blob_backend import (
    AUTH_TOKEN_ENV,
    RemoteBlobBackend,
)
from repositories.srv import (
    BlobRepositoryDeleteBlob,
    BlobRepositoryFetchBlob,
    BlobRepositoryGetBlobInfo,
    BlobRepositoryUploadBlob,
)
from repositories.utils.json_metadata import dump_metadata


class BlobRepository(LifecycleNode):
    """Blob upload/fetch/info/delete service exposed as a managed lifecycle node."""

    def __init__(self, node_name: str = 'blob_repository') -> None:
        """Declare parameters; defer all resource creation to lifecycle transitions.

        Args:
            node_name: The ROS node name. Defaults to ``'blob_repository'``.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'local_storage',
            True,
            ParameterDescriptor(
                description='true keeps blobs only on the local filesystem; false uses '
                'the cloud blob service with the local directory as a cache.',
            ),
        )
        self.declare_parameter(
            'local_storage_path',
            '/tmp/station/blobs',
            ParameterDescriptor(
                description='Absolute path of a directory dedicated to this node. '
                'Created if missing; files that are not complete blob/metadata pairs '
                'are deleted on configure.',
            ),
        )
        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(description='Required UUID4 string scoping cloud calls.'),
        )
        self.declare_parameter(
            'blob_service_url',
            '',
            ParameterDescriptor(
                description='HTTPS base URL of the cloud blob service. Required only '
                'when local_storage is false.',
            ),
        )
        self.declare_parameter(
            'request_timeout_sec',
            5.0,
            ParameterDescriptor(
                description='Timeout for connecting and for metadata requests. Must be > 0.',
            ),
        )
        self.declare_parameter(
            'transfer_timeout_sec',
            600.0,
            ParameterDescriptor(
                description='Total timeout for streaming one file. Must be > 0.',
            ),
        )
        self.declare_parameter(
            'cache_size_mb',
            3072,
            ParameterDescriptor(
                description='Upper bound in MiB for the local storage/cache directory. '
                'Must be > 0 and >= max_blob_size_mb.',
            ),
        )
        self.declare_parameter(
            'max_blob_size_mb',
            1024,
            ParameterDescriptor(
                description='Upper bound in MiB for one blob. Must be > 0 and '
                '<= cache_size_mb.',
            ),
        )
        self.declare_parameter(
            'cache_check_interval_sec',
            60.0,
            ParameterDescriptor(
                description='Period of the background cache size check while active. '
                'Must be > 0.',
            ),
        )
        self.declare_parameter(
            'allowed_path_roots',
            Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(
                description='Absolute existing directories that upload sources and '
                'explicit fetch destinations must resolve within. At least one is required.',
            ),
        )

        self._config: BlobRepositoryConfigDTO | None = None
        self._store: LocalBlobStore | None = None
        self._backend: BlobBackend | None = None
        self._backend_initialized = False
        self._manager: BlobManager | None = None

        self._callback_group = ReentrantCallbackGroup()
        # NOTE: avoid the name `_services` because rclpy Node uses it internally.
        self._blob_services: list[Service] = []
        self._blob_action_servers: list[ActionServer] = []
        self._cache_timer: Timer | None = None

    # region: lifecycle

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate parameters, prepare the store and build backend and manager.

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE (logged) if a parameter
            is invalid, an allowed root is unusable or the storage directory
            cannot be prepared. No ROS entities are created here.
        """
        try:
            config = self._read_config()
        except ValidationError as exc:
            details = '; '.join(
                f"{'.'.join(str(part) for part in error['loc']) or 'config'}: {error['msg']}"
                for error in exc.errors(include_input=False, include_url=False)
            )
            self.get_logger().error(f'Invalid configuration: {details}')
            return TransitionCallbackReturn.FAILURE

        store = LocalBlobStore(
            Path(config.local_storage_path), config.cache_size_bytes, self.get_logger()
        )
        try:
            path_policy = PathPolicy(config.allowed_path_roots, store.root)
            store.prepare()
        except BlobRepositoryError as exc:
            self.get_logger().error(f'Invalid allowed_path_roots: {exc}')
            return TransitionCallbackReturn.FAILURE
        except OSError as exc:
            self.get_logger().error(f'Cannot use local_storage_path: {exc}')
            return TransitionCallbackReturn.FAILURE

        backend: BlobBackend
        if config.local_storage:
            backend = LocalBlobBackend(store)
        else:
            remote = RemoteBlobBackend(
                station_id=config.station_id,
                blob_service_url=config.blob_service_url,
                request_timeout_sec=config.request_timeout_sec,
                transfer_timeout_sec=config.transfer_timeout_sec,
            )
            if not remote.has_auth_token:
                self.get_logger().warning(
                    f'{AUTH_TOKEN_ENV} is not set; cloud requests are sent without '
                    'an Authorization header.'
                )
            backend = remote

        self._config = config
        self._store = store
        self._backend = backend
        self._manager = BlobManager(
            store, backend, path_policy, config.max_blob_size_bytes, self.get_logger()
        )
        mode = 'local' if config.local_storage else 'cloud'
        self.get_logger().info(
            f"Configured blob_repository ({mode} mode, path='{config.local_storage_path}')."
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Initialize the backend, create the services, actions and cache timer.

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE if the node is not
            configured or activation fails.
        """
        if self._config is None or self._manager is None or self._backend is None:
            self.get_logger().error('Cannot activate: node is not configured.')
            return TransitionCallbackReturn.FAILURE

        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_activate did not succeed; aborting.')
            return TransitionCallbackReturn.FAILURE

        try:
            self._backend.initialize()
            self._backend_initialized = True
            self._create_ros_surface()
            self._cache_timer = self.create_timer(
                self._config.cache_check_interval_sec,
                self._on_cache_timer,
                callback_group=self._callback_group,
            )
        except Exception as exc:  # noqa: BLE001 - leave the node inactive
            self.get_logger().error(f'Activation failed: {exc}')
            self._destroy_entities()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Activated blob_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destroy the services, actions and timer and shut the backend down.

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE if teardown or the base
            deactivation fails.
        """
        ok = self._destroy_entities()
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_deactivate did not succeed.')
            return TransitionCallbackReturn.FAILURE
        if not ok:
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Deactivated blob_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop config, backend, store and manager; persisted blobs are kept.

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE if teardown fails.
        """
        if not self.release_resources():
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Cleaned up blob_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release everything from any primary state; persisted blobs are kept.

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE if teardown fails.
        """
        if not self.release_resources():
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Shut down blob_repository.')
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> bool:
        """Destroy ROS entities, shut the backend down and drop in-memory state.

        Idempotent and safe after deactivation or cleanup. Never deletes
        persisted blobs.

        Returns:
            bool: False if any teardown step failed (each failure is logged).
        """
        ok = self._destroy_entities()
        self._manager = None
        self._store = None
        self._backend = None
        self._backend_initialized = False
        self._config = None
        return ok

    def destroy_node(self) -> None:
        """Release resources (even without a lifecycle shutdown) and destroy the node."""
        self.release_resources()
        super().destroy_node()

    # endregion: lifecycle

    # region: ROS surface

    def _create_ros_surface(self) -> None:
        """Create the node-private services and batch action servers."""
        group = self._callback_group
        self._blob_services = [
            self.create_service(
                BlobRepositoryUploadBlob, '~/upload_blob', self._on_upload_blob,
                callback_group=group,
            ),
            self.create_service(
                BlobRepositoryFetchBlob, '~/fetch_blob', self._on_fetch_blob,
                callback_group=group,
            ),
            self.create_service(
                BlobRepositoryGetBlobInfo, '~/get_blob_info', self._on_get_blob_info,
                callback_group=group,
            ),
            self.create_service(
                BlobRepositoryDeleteBlob, '~/delete_blob', self._on_delete_blob,
                callback_group=group,
            ),
        ]
        self._blob_action_servers = [
            ActionServer(
                self,
                BlobRepositoryUploadBlobs,
                '~/upload_blobs',
                execute_callback=self._execute_upload_blobs,
                cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
                callback_group=group,
            ),
            ActionServer(
                self,
                BlobRepositoryFetchBlobs,
                '~/fetch_blobs',
                execute_callback=self._execute_fetch_blobs,
                cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
                callback_group=group,
            ),
        ]

    def _destroy_entities(self) -> bool:
        """Destroy services, action servers and timer, then shut the backend down.

        Returns:
            bool: False if any step failed (each failure is logged).
        """
        ok = True
        for service in self._blob_services:
            try:
                self.destroy_service(service)
            except Exception as exc:  # noqa: BLE001 - keep tearing down
                self.get_logger().error(f'destroying service failed: {exc}')
                ok = False
        self._blob_services = []
        for server in self._blob_action_servers:
            try:
                server.destroy()
            except Exception as exc:  # noqa: BLE001 - keep tearing down
                self.get_logger().error(f'destroying action server failed: {exc}')
                ok = False
        self._blob_action_servers = []
        if self._cache_timer is not None:
            try:
                self._cache_timer.cancel()
                self.destroy_timer(self._cache_timer)
            except Exception as exc:  # noqa: BLE001 - keep tearing down
                self.get_logger().error(f'destroying cache timer failed: {exc}')
                ok = False
            self._cache_timer = None
        if self._backend is not None and self._backend_initialized:
            self._backend_initialized = False
            try:
                self._backend.shutdown()
            except Exception as exc:  # noqa: BLE001 - keep tearing down
                self.get_logger().error(f'backend shutdown failed: {exc}')
                ok = False
        return ok

    # endregion: ROS surface

    # region: service handlers

    def _on_upload_blob(
        self,
        request: BlobRepositoryUploadBlob.Request,
        response: BlobRepositoryUploadBlob.Response,
    ) -> BlobRepositoryUploadBlob.Response:
        """Upload one file; never raises across the ROS boundary."""
        self._reset_upload_response(response)
        try:
            item = BlobUploadItemDTO.from_request(
                request.source_path,
                request.blob_id,
                request.blob_name,
                request.media_type,
                request.metadata,
            )
            descriptor = self._require_manager().upload([item])[0]
            response.blob_id = descriptor.blob_id
            response.blob_name = descriptor.blob_name
            response.media_type = descriptor.media_type
            response.size_bytes = descriptor.size_bytes
            response.sha256 = descriptor.sha256
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            self._reset_upload_response(response)
            response.error_message = self._failure_message('upload_blob', exc)
        return response

    def _on_fetch_blob(
        self,
        request: BlobRepositoryFetchBlob.Request,
        response: BlobRepositoryFetchBlob.Response,
    ) -> BlobRepositoryFetchBlob.Response:
        """Fetch one blob to the cache or a destination; never raises across the boundary."""
        self._reset_fetch_response(response)
        try:
            destinations = [request.destination_path] if request.destination_path else []
            result = self._require_manager().fetch(
                [request.blob_id], destinations, request.download_fresh
            )[0]
            response.local_path = str(result.local_path)
            self._fill_descriptor(response, result.descriptor)
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            self._reset_fetch_response(response)
            response.error_message = self._failure_message('fetch_blob', exc)
        return response

    def _on_get_blob_info(
        self,
        request: BlobRepositoryGetBlobInfo.Request,
        response: BlobRepositoryGetBlobInfo.Response,
    ) -> BlobRepositoryGetBlobInfo.Response:
        """Return a descriptor; never raises across the boundary."""
        self._reset_info_response(response)
        try:
            self._fill_descriptor(response, self._require_manager().info(request.blob_id))
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            self._reset_info_response(response)
            response.error_message = self._failure_message('get_blob_info', exc)
        return response

    def _on_delete_blob(
        self,
        request: BlobRepositoryDeleteBlob.Request,
        response: BlobRepositoryDeleteBlob.Response,
    ) -> BlobRepositoryDeleteBlob.Response:
        """Delete one blob; never raises across the boundary."""
        response.success = False
        response.error_message = ''
        try:
            self._require_manager().delete(request.blob_id)
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            response.error_message = self._failure_message('delete_blob', exc)
        return response

    def _on_cache_timer(self) -> None:
        """Timer callback: keep the local directory within the cache limit."""
        manager = self._manager
        if manager is None:
            return
        try:
            manager.enforce_cache_limit()
        except Exception as exc:  # noqa: BLE001 - never let the timer crash
            self.get_logger().error(f'cache size check failed: {exc}')

    # endregion: service handlers

    # region: action handlers

    def _execute_upload_blobs(
        self, goal_handle: ServerGoalHandle
    ) -> BlobRepositoryUploadBlobs.Result:
        """Upload several files all-or-nothing with per-item feedback.

        Returns:
            The goal result; never raises across the boundary.
        """
        result = BlobRepositoryUploadBlobs.Result()
        result.success = False
        result.blob_ids = []
        result.error_message = ''
        try:
            items = self._upload_items_from_goal(goal_handle.request)
            total = len(items)

            def publish(
                index: int, item: BlobUploadItemDTO, descriptor: BlobDescriptorDTO
            ) -> None:
                feedback = BlobRepositoryUploadBlobs.Feedback()
                feedback.completed = index + 1
                feedback.total = total
                feedback.source_path = item.source_path
                feedback.blob_id = descriptor.blob_id
                goal_handle.publish_feedback(feedback)

            descriptors = self._require_manager().upload(
                items, publish, lambda: goal_handle.is_cancel_requested
            )
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            result.error_message = self._failure_message('upload_blobs', exc)
            self._finish_failed_goal(goal_handle, exc)
            return result

        goal_handle.succeed()
        result.success = True
        result.blob_ids = [descriptor.blob_id for descriptor in descriptors]
        return result

    def _execute_fetch_blobs(
        self, goal_handle: ServerGoalHandle
    ) -> BlobRepositoryFetchBlobs.Result:
        """Fetch several blobs fail-fast with per-item feedback.

        Returns:
            The goal result; never raises across the boundary.
        """
        result = BlobRepositoryFetchBlobs.Result()
        result.success = False
        result.local_paths = []
        result.error_message = ''
        try:
            request = goal_handle.request
            blob_ids = list(request.blob_ids)
            total = len(blob_ids)

            def publish(index: int, blob_id: str, fetched: BlobFetchResultDTO) -> None:
                feedback = BlobRepositoryFetchBlobs.Feedback()
                feedback.completed = index + 1
                feedback.total = total
                feedback.blob_id = blob_id
                feedback.local_path = str(fetched.local_path)
                goal_handle.publish_feedback(feedback)

            fetched = self._require_manager().fetch(
                blob_ids,
                list(request.destination_paths),
                request.download_fresh,
                publish,
                lambda: goal_handle.is_cancel_requested,
            )
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            result.error_message = self._failure_message('fetch_blobs', exc)
            self._finish_failed_goal(goal_handle, exc)
            return result

        goal_handle.succeed()
        result.success = True
        result.local_paths = [str(item.local_path) for item in fetched]
        return result

    # endregion: action handlers

    # region: helpers

    def _read_config(self) -> BlobRepositoryConfigDTO:
        """Capture the current parameter values in a validated DTO.

        Raises:
            ValidationError: If a value is invalid.
        """
        try:
            roots = list(
                self.get_parameter('allowed_path_roots').get_parameter_value().string_array_value
            )
        except ParameterUninitializedException:
            roots = []
        return BlobRepositoryConfigDTO(
            local_storage=self._param('local_storage').bool_value,
            local_storage_path=self._param('local_storage_path').string_value,
            station_id=self._param('station_id').string_value,
            blob_service_url=self._param('blob_service_url').string_value,
            request_timeout_sec=self._param('request_timeout_sec').double_value,
            transfer_timeout_sec=self._param('transfer_timeout_sec').double_value,
            cache_size_mb=self._param('cache_size_mb').integer_value,
            max_blob_size_mb=self._param('max_blob_size_mb').integer_value,
            cache_check_interval_sec=self._param('cache_check_interval_sec').double_value,
            allowed_path_roots=tuple(roots),
        )

    def _param(self, name: str) -> ParameterValue:
        return self.get_parameter(name).get_parameter_value()

    @staticmethod
    def _upload_items_from_goal(
        goal: BlobRepositoryUploadBlobs.Goal,
    ) -> list[BlobUploadItemDTO]:
        """Validate the goal arrays and build upload items (no side effects).

        Raises:
            BlobRepositoryError: If the batch is empty, an optional array has the
                wrong length or an element is malformed.
        """
        sources = list(goal.source_paths)
        if not sources:
            raise BlobRepositoryError('source_paths must not be empty')
        optional = {
            'blob_ids': list(goal.blob_ids),
            'blob_names': list(goal.blob_names),
            'media_types': list(goal.media_types),
            'metadata': list(goal.metadata),
        }
        for name, values in optional.items():
            if values and len(values) != len(sources):
                raise BlobRepositoryError(
                    f'{name} must be empty or have the same length as source_paths'
                )

        def element(name: str, index: int) -> str:
            values = optional[name]
            return values[index] if values else ''

        items = []
        for index, source in enumerate(sources):
            try:
                items.append(
                    BlobUploadItemDTO.from_request(
                        source,
                        element('blob_ids', index),
                        element('blob_names', index),
                        element('media_types', index),
                        element('metadata', index),
                    )
                )
            except ValueError as exc:
                raise BlobRepositoryError(f'item {index}: {exc}') from exc
        return items

    def _finish_failed_goal(self, goal_handle: ServerGoalHandle, exc: Exception) -> None:
        if isinstance(exc, BlobOperationCanceledError) and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

    def _require_manager(self) -> BlobManager:
        manager = self._manager
        if manager is None:
            raise BlobRepositoryError('node is not configured')
        return manager

    def _failure_message(self, operation: str, exc: Exception) -> str:
        """Log a failed request and return the message for the response."""
        message = str(exc) or type(exc).__name__
        self.get_logger().error(f'{operation} failed: {message}')
        return message

    @staticmethod
    def _fill_descriptor(response, descriptor: BlobDescriptorDTO) -> None:
        response.blob_name = descriptor.blob_name
        response.media_type = descriptor.media_type
        response.size_bytes = descriptor.size_bytes
        response.sha256 = descriptor.sha256
        response.metadata = dump_metadata(descriptor.metadata)

    @staticmethod
    def _reset_upload_response(response: BlobRepositoryUploadBlob.Response) -> None:
        response.blob_id = ''
        response.blob_name = ''
        response.media_type = ''
        response.size_bytes = 0
        response.sha256 = ''
        response.success = False
        response.error_message = ''

    @staticmethod
    def _reset_info_response(response) -> None:
        response.blob_name = ''
        response.media_type = ''
        response.size_bytes = 0
        response.sha256 = ''
        response.metadata = ''
        response.success = False
        response.error_message = ''

    @classmethod
    def _reset_fetch_response(cls, response: BlobRepositoryFetchBlob.Response) -> None:
        cls._reset_info_response(response)
        response.local_path = ''

    # endregion: helpers


def main(args: list[str] | None = None) -> None:
    """Run the ``blob_repository`` node; resources are released on any exit."""
    rclpy.init(args=args)
    node = BlobRepository()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
