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
"""Object model storage lifecycle node.

``object_model_storage`` provides 3D object model metadata lookup and local
model file caching to other nodes. Model metadata is always read afresh from
the configured source (``local_storage.json`` in local mode, the storage REST
service in cloud mode) while model files are cached in ``download_path`` under
their UUID4 ``model_id``. Configuration is frozen on ``configure``; services,
the ``get_models`` action and the latched ``cached_models`` topic exist only
while the node is ``active``.
"""

import threading
from pathlib import Path

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.service import Service

from repositories.action import ObjectModelStorageGetModels
from repositories.interfaces.object_model_storage.model_storage_backend import (
    ModelStorageBackend,
)
from repositories.models.object_model_storage.object_model_info_dto import (
    MappingValueDTO,
    ObjectModelInfoDTO,
)
from repositories.models.object_model_storage.object_model_storage_config_dto import (
    ObjectModelStorageConfigDTO,
)
from repositories.msg import ObjectModelStorageCachedModels
from repositories.services.object_model_storage.cloud_model_storage_backend import (
    CloudModelStorageBackend,
)
from repositories.services.object_model_storage.local_model_storage_backend import (
    LocalModelStorageBackend,
)
from repositories.services.object_model_storage.model_cache import ModelCache
from repositories.srv import ObjectModelStorageGetModel, ObjectModelStorageGetModelsInfo
from repositories.utils.uuid_validator import parse_uuid4


class ObjectModelStorage(LifecycleNode):
    """Object model metadata and local model cache exposed as a lifecycle node."""

    def __init__(self, node_name: str = 'object_model_storage') -> None:
        """Declare parameters; defer all resource creation to lifecycle transitions.

        Args:
            node_name: The ROS node name. Defaults to ``'object_model_storage'``.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'local_storage',
            True,
            ParameterDescriptor(
                description='true reads model metadata from '
                '<download_path>/local_storage.json and never uses the network; '
                'false uses the cloud storage REST service (currently stubbed).',
            ),
        )
        self.declare_parameter(
            'download_path',
            '/tmp/station/object_models/',
            ParameterDescriptor(
                description='Absolute directory holding cached model files/'
                'folders named by model_id; created on configure if missing.',
            ),
        )
        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(
                description='Required station scope used in cloud REST paths. '
                'Must be a UUID4 string; configure fails otherwise.',
            ),
        )
        self.declare_parameter(
            'storage_service_url',
            '',
            ParameterDescriptor(
                description='Base URL of the model storage REST service. '
                'Required only when local_storage is false.',
            ),
        )
        self.declare_parameter(
            'request_timeout_sec',
            5.0,
            ParameterDescriptor(
                description='HTTP timeout for cloud calls (provisional; unused '
                'until cloud mode is implemented). Must be > 0.',
            ),
        )

        # Frozen configuration, model source and cache index; set on configure.
        self._config: ObjectModelStorageConfigDTO | None = None
        self._backend: ModelStorageBackend | None = None
        self._cache: ModelCache | None = None

        # Latest metadata seen from the source, used only to describe cached
        # models on the cached_models topic (never to answer metadata queries).
        self._known_models: dict[str, ObjectModelInfoDTO] = {}

        # Serializes cache-miss downloads so a model is fetched at most once.
        self._download_lock = threading.Lock()

        # Live ROS entities, created on activate and destroyed on deactivate.
        # NOTE: avoid the name `_services` because rclpy Node uses it internally.
        self._callback_group = ReentrantCallbackGroup()
        self._model_services: list[Service] = []
        self._get_models_server: ActionServer | None = None
        self._cached_models_pub: Publisher | None = None
        self._is_active = False

    # region: lifecycle

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate parameters, freeze config, prepare the cache and the backend.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE on any invalid
            parameter or if ``download_path`` cannot be created/scanned.
        """
        try:
            config = ObjectModelStorageConfigDTO(
                local_storage=self.get_parameter('local_storage')
                .get_parameter_value()
                .bool_value,
                download_path=self.get_parameter('download_path')
                .get_parameter_value()
                .string_value,
                station_id=self.get_parameter('station_id')
                .get_parameter_value()
                .string_value,
                storage_service_url=self.get_parameter('storage_service_url')
                .get_parameter_value()
                .string_value,
                request_timeout_sec=self.get_parameter('request_timeout_sec')
                .get_parameter_value()
                .double_value,
            )
        except ValidationError as exc:
            self.get_logger().error(f'Invalid configuration: {exc}')
            return TransitionCallbackReturn.FAILURE

        download_path = Path(config.download_path)
        cache = ModelCache(download_path)
        try:
            cache.ensure_root()
            cached_count = cache.scan()
        except OSError as exc:
            self.get_logger().error(
                f"Cannot prepare download_path '{download_path}': {exc}"
            )
            return TransitionCallbackReturn.FAILURE

        backend: ModelStorageBackend
        if config.local_storage:
            local_backend = LocalModelStorageBackend(download_path)
            if not local_backend.local_storage_file.is_file():
                self.get_logger().warning(
                    f"Local metadata file '{local_backend.local_storage_file}' "
                    'not found; model queries will fail until it is created.'
                )
            backend = local_backend
        else:
            backend = CloudModelStorageBackend(
                station_id=config.station_id,
                storage_service_url=config.storage_service_url,
                request_timeout_sec=config.request_timeout_sec,
            )

        self._config = config
        self._cache = cache
        self._backend = backend
        self._known_models = {}

        mode = 'local' if config.local_storage else 'cloud (stubbed)'
        self.get_logger().info(
            f"Configured object_model_storage (station_id='{config.station_id}', "
            f"{mode} mode, download_path='{download_path}', "
            f'{cached_count} cached models).'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create services, the action server and the latched topic; publish once.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE if the node is
            not configured or the base activation does not succeed.
        """
        if self._backend is None or self._cache is None:
            self.get_logger().error('Cannot activate: node is not configured.')
            return TransitionCallbackReturn.FAILURE

        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_activate did not succeed; aborting.')
            return TransitionCallbackReturn.FAILURE

        self._create_ros_surface()
        self._is_active = True

        self._refresh_known_models()
        self._publish_cached_models()

        self.get_logger().info(
            'Activated object_model_storage (services: ~/get_models_info, '
            '~/get_model; action: ~/get_models; topic: ~/cached_models).'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destroy services, the action server and the topic; keep the cache index.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE if the base
            deactivation does not succeed.
        """
        self._is_active = False
        self._destroy_ros_surface()

        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_deactivate did not succeed.')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Deactivated object_model_storage (cache retained).')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop config, backend and cache index; files on disk are kept.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Cleaned up object_model_storage.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Tear down all resources on shutdown.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Shut down object_model_storage.')
        return TransitionCallbackReturn.SUCCESS

    # endregion: lifecycle

    # region: ROS surface

    def _create_ros_surface(self) -> None:
        """Create the node-relative services, action server and latched topic."""
        self._model_services = [
            self.create_service(
                ObjectModelStorageGetModelsInfo,
                '~/get_models_info',
                self._on_get_models_info,
                callback_group=self._callback_group,
            ),
            self.create_service(
                ObjectModelStorageGetModel,
                '~/get_model',
                self._on_get_model,
                callback_group=self._callback_group,
            ),
        ]
        self._get_models_server = ActionServer(
            self,
            ObjectModelStorageGetModels,
            '~/get_models',
            execute_callback=self._execute_get_models,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self._cached_models_pub = self.create_publisher(
            ObjectModelStorageCachedModels,
            '~/cached_models',
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        )

    def _destroy_ros_surface(self) -> None:
        """Destroy every live service, the action server and the publisher."""
        for service in self._model_services:
            self.destroy_service(service)
        self._model_services = []
        if self._get_models_server is not None:
            self._get_models_server.destroy()
            self._get_models_server = None
        if self._cached_models_pub is not None:
            self.destroy_publisher(self._cached_models_pub)
            self._cached_models_pub = None

    # endregion: ROS surface

    # region: service / action handlers

    def _on_get_models_info(
        self,
        request: ObjectModelStorageGetModelsInfo.Request,
        response: ObjectModelStorageGetModelsInfo.Response,
    ) -> ObjectModelStorageGetModelsInfo.Response:
        """Return every model carrying ALL requested ``filter_keys`` pairs.

        Args:
            request: Request carrying the ``filter_keys`` to match.
            response: Response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        response.models = []
        try:
            filters = [
                MappingValueDTO(key=item.key, value=item.value)
                for item in request.filter_keys
            ]
            matched = [
                model for model in self._fetch_models_info()
                if model.matches_all(filters)
            ]
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            message = f'failed to get models info: {exc}'
            self.get_logger().error(message)
            response.success = False
            response.error_message = message
            return response

        response.models = [model.to_msg() for model in matched]
        response.success = True
        response.error_message = ''
        self.get_logger().info(
            f'get_models_info: {len(matched)} models match '
            f'{len(filters)} filter keys.'
        )
        return response

    def _on_get_model(
        self,
        request: ObjectModelStorageGetModel.Request,
        response: ObjectModelStorageGetModel.Response,
    ) -> ObjectModelStorageGetModel.Response:
        """Make one model available locally and return its absolute path.

        Args:
            request: Request carrying the ``model_id`` to resolve.
            response: Response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        try:
            path, _ = self._resolve_model(request.model_id)
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            message = f"failed to get model '{request.model_id}': {exc}"
            self.get_logger().error(message)
            response.local_path = ''
            response.success = False
            response.error_message = message
            return response

        response.local_path = str(path)
        response.success = True
        response.error_message = ''
        self.get_logger().info(f"get_model '{request.model_id}' -> '{path}'.")
        self._publish_cached_models()
        return response

    def _execute_get_models(
        self, goal_handle: ServerGoalHandle
    ) -> ObjectModelStorageGetModels.Result:
        """Resolve several models one-by-one, publishing feedback per model.

        Processing is fail-fast: the first invalid id or failed model aborts
        the goal. A cancel request is honored between models.

        Args:
            goal_handle: Handle of the accepted ``get_models`` goal.

        Returns:
            The goal result (``success`` + ``error_message``); never raises
            across the boundary.
        """
        model_ids = list(goal_handle.request.model_ids)
        result = ObjectModelStorageGetModels.Result()
        cache_changed = False

        def fail(message: str, canceled: bool = False) -> ObjectModelStorageGetModels.Result:
            self.get_logger().error(f'get_models: {message}')
            if canceled:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            if cache_changed:
                self._publish_cached_models()
            result.success = False
            result.error_message = message
            return result

        if not model_ids:
            return fail('model_ids must not be empty')
        try:
            for model_id in model_ids:
                parse_uuid4(model_id, 'model_id')
        except ValueError as exc:
            return fail(str(exc))

        self.get_logger().info(f'get_models: processing {len(model_ids)} models.')
        for index, model_id in enumerate(model_ids):
            if goal_handle.is_cancel_requested:
                return fail(
                    f'canceled after {index} of {len(model_ids)} models',
                    canceled=True,
                )
            if not self._is_active:
                return fail('node was deactivated during processing')

            feedback = ObjectModelStorageGetModels.Feedback()
            feedback.model_id = model_id
            try:
                path, newly_cached = self._resolve_model(model_id)
            except Exception as exc:  # noqa: BLE001 - convert to failure result
                feedback.local_path = ''
                goal_handle.publish_feedback(feedback)
                return fail(f"model '{model_id}' failed: {exc}")

            cache_changed = cache_changed or newly_cached
            feedback.local_path = str(path)
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self._publish_cached_models()
        result.success = True
        result.error_message = ''
        self.get_logger().info(f'get_models: all {len(model_ids)} models available.')
        return result

    # endregion: service / action handlers

    # region: helpers

    def _fetch_models_info(self) -> list[ObjectModelInfoDTO]:
        """Fetch all model metadata from the backend and remember it.

        Returns:
            list[ObjectModelInfoDTO]: Every model known to the source.

        Raises:
            RuntimeError: If the node is not configured.
            Exception: Any backend failure (including the cloud stub).
        """
        backend = self._backend
        if backend is None:
            raise RuntimeError('node is not active')
        models = backend.fetch_models_info()
        self._known_models = {model.model_id: model for model in models}
        return models

    def _resolve_model(self, raw_model_id: str) -> tuple[Path, bool]:
        """Return the cached path of a model, downloading it if necessary.

        Args:
            raw_model_id: The requested model id (validated here).

        Returns:
            tuple[Path, bool]: The absolute cached path, and whether the model
            was newly added to the cache by this call.

        Raises:
            ValueError: If the id is not a canonical UUID4.
            LookupError: If the model is unknown to the metadata source.
            Exception: Any backend failure (missing file, cloud stub, ...).
        """
        cache = self._cache
        backend = self._backend
        if cache is None or backend is None:
            raise RuntimeError('node is not active')

        model_id = parse_uuid4(raw_model_id, 'model_id')
        if not any(model.model_id == model_id for model in self._fetch_models_info()):
            raise LookupError(f"model '{model_id}' not found in model storage")

        # Double-checked locking: the unlocked lookup keeps cache hits from
        # waiting behind an in-progress download; the re-check under the lock
        # catches a concurrent request that downloaded this same model while
        # we waited, so it is downloaded only once.
        cached = cache.get(model_id)
        if cached is not None:
            return cached, False

        with self._download_lock:
            cached = cache.get(model_id)
            if cached is not None:
                return cached, False
            target = cache.target_path(model_id)
            if not target.exists():
                backend.download_model(model_id, target)
            return cache.register(model_id), True

    def _refresh_known_models(self) -> None:
        """Best-effort metadata refresh used to describe cached models."""
        try:
            self._fetch_models_info()
        except Exception as exc:  # noqa: BLE001 - snapshot falls back to known data
            self.get_logger().warning(
                f'Could not read model metadata for cached_models: {exc}'
            )

    def _publish_cached_models(self) -> None:
        """Publish the latched snapshot of cached models with known metadata."""
        publisher = self._cached_models_pub
        cache = self._cache
        if publisher is None or cache is None:
            return
        known = self._known_models
        cached_ids = cache.model_ids()
        msg = ObjectModelStorageCachedModels()
        msg.models = [
            known[model_id].to_msg() for model_id in cached_ids if model_id in known
        ]
        publisher.publish(msg)
        self.get_logger().debug(
            f'Published cached_models ({len(msg.models)} of {len(cached_ids)} '
            'cached models have metadata).'
        )

    def _teardown(self) -> None:
        """Release the ROS surface, cache index, backend and config."""
        self._is_active = False
        self._destroy_ros_surface()
        if self._cache is not None:
            self._cache.clear()
            self._cache = None
        self._backend = None
        self._known_models = {}
        self._config = None

    # endregion: helpers


def main(args: list[str] | None = None) -> None:
    """Spin the object model storage lifecycle node until shutdown.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = ObjectModelStorage()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
