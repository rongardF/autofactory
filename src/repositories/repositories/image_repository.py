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
"""Image upload/fetch lifecycle node.

``image_repository`` lets other nodes upload images to, and fetch images from, a
storage. Every image is kept on the local filesystem (size-limited, oldest
images evicted first); in remote mode the remote image service is the source of
truth and the local directory is a cache. The node owns the ROS service surface
and parameter handling and delegates storage to :class:`ImageManager`.
Configuration is frozen on ``configure``; services exist only while ``active``.
"""

from pathlib import Path

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.service import Service
from rclpy.timer import Timer

from repositories.exceptions.image_repository import ImageRepositoryError
from repositories.interfaces.image_repository.image_backend import ImageBackend
from repositories.models.image_repository.image_payload_dto import ImagePayloadDTO
from repositories.models.image_repository.image_repository_config_dto import (
    ImageRepositoryConfigDTO,
)
from repositories.services.image_repository.image_manager import ImageManager
from repositories.services.image_repository.local_image_store import LocalImageStore
from repositories.services.image_repository.remote_image_backend import RemoteImageBackend
from repositories.srv import (
    ImageRepositoryFetchImage,
    ImageRepositoryFetchImages,
    ImageRepositoryUploadImage,
    ImageRepositoryUploadImages,
)
from repositories.utils.json_metadata import dump_metadata


class ImageRepository(LifecycleNode):
    """Image upload/fetch service exposed as a managed lifecycle node."""

    def __init__(self, node_name: str = 'image_repository') -> None:
        """Declare parameters; defer all resource creation to lifecycle transitions.

        Args:
            node_name: The ROS node name. Defaults to ``'image_repository'``.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'local_storage',
            True,
            ParameterDescriptor(
                description='true keeps images only on the local filesystem; false '
                'uses the remote image service with the local directory as a cache.',
            ),
        )
        self.declare_parameter(
            'local_storage_path',
            '/tmp/station/images',
            ParameterDescriptor(
                description='Absolute path of a directory dedicated to this node. '
                'Created if missing; files that are not image/metadata pairs are '
                'deleted on configure.',
            ),
        )
        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(
                description='Required UUID4 string used to namespace remote calls.',
            ),
        )
        self.declare_parameter(
            'image_service_url',
            '',
            ParameterDescriptor(
                description='Base URL of the remote image service. Required only '
                'when local_storage is false.',
            ),
        )
        self.declare_parameter(
            'request_timeout_sec',
            5.0,
            ParameterDescriptor(
                description='HTTP timeout for remote calls. Must be > 0.',
            ),
        )
        self.declare_parameter(
            'cache_size_mb',
            50,
            ParameterDescriptor(
                description='Upper bound in MB for the local directory size; '
                'eviction starts 10 MB below it. Must be > 10.',
            ),
        )
        self.declare_parameter(
            'cache_check_interval_sec',
            60.0,
            ParameterDescriptor(
                description='Period of the background cache size check while '
                'active. Must be > 0.',
            ),
        )

        self._config: ImageRepositoryConfigDTO | None = None
        self._store: LocalImageStore | None = None
        self._backend: ImageBackend | None = None
        self._manager: ImageManager | None = None

        self._callback_group = ReentrantCallbackGroup()
        # NOTE: avoid the name `_services` because rclpy Node uses it internally.
        self._image_services: list[Service] = []
        self._cache_timer: Timer | None = None

    # region: lifecycle

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate parameters, prepare the local store and build the manager.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE (logged) if a parameter
            is invalid or the storage directory is unusable. No ROS entities are
            created here.
        """
        try:
            config = ImageRepositoryConfigDTO(
                local_storage=self.get_parameter('local_storage').get_parameter_value().bool_value,
                local_storage_path=self.get_parameter('local_storage_path')
                .get_parameter_value()
                .string_value,
                station_id=self.get_parameter('station_id').get_parameter_value().string_value,
                image_service_url=self.get_parameter('image_service_url')
                .get_parameter_value()
                .string_value,
                request_timeout_sec=self.get_parameter('request_timeout_sec')
                .get_parameter_value()
                .double_value,
                cache_size_mb=self.get_parameter('cache_size_mb')
                .get_parameter_value()
                .integer_value,
                cache_check_interval_sec=self.get_parameter('cache_check_interval_sec')
                .get_parameter_value()
                .double_value,
            )
        except ValidationError as exc:
            self.get_logger().error(f'Invalid configuration: {exc}')
            return TransitionCallbackReturn.FAILURE

        store = LocalImageStore(
            Path(config.local_storage_path), config.threshold_bytes, self.get_logger()
        )
        try:
            store.prepare()
        except OSError as exc:
            self.get_logger().error(f'Cannot use local_storage_path: {exc}')
            return TransitionCallbackReturn.FAILURE

        backend: ImageBackend | None = None
        if not config.local_storage:
            backend = RemoteImageBackend(
                station_id=config.station_id,
                image_service_url=config.image_service_url,
                request_timeout_sec=config.request_timeout_sec,
            )

        self._config = config
        self._store = store
        self._backend = backend
        self._manager = ImageManager(store, backend, self.get_logger())

        mode = 'local' if config.local_storage else 'remote (stubbed)'
        self.get_logger().info(
            f"Configured image_repository ({mode} mode, path='{config.local_storage_path}')."
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Initialize the backend, create the services and the cache timer.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE if the node is not
            configured or activation fails.
        """
        if self._config is None or self._manager is None:
            self.get_logger().error('Cannot activate: node is not configured.')
            return TransitionCallbackReturn.FAILURE

        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_activate did not succeed; aborting.')
            return TransitionCallbackReturn.FAILURE

        try:
            if self._backend is not None:
                self._backend.initialize()
            self._create_services()
            self._cache_timer = self.create_timer(
                self._config.cache_check_interval_sec,
                self._on_cache_timer,
                callback_group=self._callback_group,
            )
        except Exception as exc:  # noqa: BLE001 - leave the node inactive
            self.get_logger().error(f'Activation failed: {exc}')
            self._destroy_entities()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Activated image_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destroy the services and the timer and de-initialize the backend.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS, or FAILURE if the base
            deactivation does not succeed.
        """
        self._destroy_entities()
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_deactivate did not succeed.')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Deactivated image_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop config, backend, store and manager; files on disk are kept.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Cleaned up image_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release everything on shutdown.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Shut down image_repository.')
        return TransitionCallbackReturn.SUCCESS

    # endregion: lifecycle

    # region: handlers

    def _create_services(self) -> None:
        """Create the four node-private services on the reentrant callback group."""
        group = self._callback_group
        self._image_services = [
            self.create_service(
                ImageRepositoryUploadImage, '~/upload_image', self._on_upload_image,
                callback_group=group,
            ),
            self.create_service(
                ImageRepositoryUploadImages, '~/upload_images', self._on_upload_images,
                callback_group=group,
            ),
            self.create_service(
                ImageRepositoryFetchImage, '~/fetch_image', self._on_fetch_image,
                callback_group=group,
            ),
            self.create_service(
                ImageRepositoryFetchImages, '~/fetch_images', self._on_fetch_images,
                callback_group=group,
            ),
        ]

    def _on_upload_image(
        self,
        request: ImageRepositoryUploadImage.Request,
        response: ImageRepositoryUploadImage.Response,
    ) -> ImageRepositoryUploadImage.Response:
        """Store one image.

        Args:
            request: The upload request.
            response: The response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        response.success = False
        response.image_id = ''
        response.error_message = ''
        try:
            payload = ImagePayloadDTO.from_image_msg(
                request.image, request.image_name, request.metadata
            )
            response.image_id = self._require_manager().upload([payload])[0]
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            response.image_id = ''
            response.error_message = self._failure_message('upload_image', exc)
        return response

    def _on_upload_images(
        self,
        request: ImageRepositoryUploadImages.Request,
        response: ImageRepositoryUploadImages.Response,
    ) -> ImageRepositoryUploadImages.Response:
        """Store several images all-or-nothing.

        Args:
            request: The upload request.
            response: The response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        response.success = False
        response.image_ids = []
        response.error_message = ''
        try:
            count = len(request.images)
            if count == 0:
                raise ImageRepositoryError('images array is empty')
            if len(request.image_names) != count or len(request.metadata) != count:
                raise ImageRepositoryError(
                    'images, image_names and metadata must have the same length'
                )
            payloads = []
            for index in range(count):
                try:
                    payloads.append(
                        ImagePayloadDTO.from_image_msg(
                            request.images[index],
                            request.image_names[index],
                            request.metadata[index],
                        )
                    )
                except ValueError as exc:
                    raise ValueError(f'image {index}: {exc}') from exc
            response.image_ids = self._require_manager().upload(payloads)
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            response.image_ids = []
            response.error_message = self._failure_message('upload_images', exc)
        return response

    def _on_fetch_image(
        self,
        request: ImageRepositoryFetchImage.Request,
        response: ImageRepositoryFetchImage.Response,
    ) -> ImageRepositoryFetchImage.Response:
        """Return one stored image.

        Args:
            request: The fetch request.
            response: The response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        response.success = False
        response.image = []
        response.image_name = []
        response.metadata = ''
        response.error_message = ''
        try:
            payload = self._require_manager().fetch([request.image_id])[0]
            image = payload.to_image_msg()
            response.image = [image]
            response.image_name = [payload.image_name]
            response.metadata = dump_metadata(payload.metadata)
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            response.image = []
            response.image_name = []
            response.metadata = ''
            response.error_message = self._failure_message('fetch_image', exc)
        return response

    def _on_fetch_images(
        self,
        request: ImageRepositoryFetchImages.Request,
        response: ImageRepositoryFetchImages.Response,
    ) -> ImageRepositoryFetchImages.Response:
        """Return several stored images all-or-nothing.

        Args:
            request: The fetch request.
            response: The response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        response.success = False
        response.images = []
        response.image_names = []
        response.returned_ids = []
        response.metadata = []
        response.error_message = ''
        try:
            requested = list(request.image_ids)
            payloads = self._require_manager().fetch(requested)
            images = [payload.to_image_msg() for payload in payloads]
            response.images = images
            response.image_names = [payload.image_name for payload in payloads]
            response.returned_ids = requested
            response.metadata = [dump_metadata(payload.metadata) for payload in payloads]
            response.success = True
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            response.images = []
            response.image_names = []
            response.returned_ids = []
            response.metadata = []
            response.error_message = self._failure_message('fetch_images', exc)
        return response

    def _on_cache_timer(self) -> None:
        """Timer callback: keep the local directory below the cache threshold."""
        if self._manager is None:
            return
        try:
            self._manager.enforce_cache_limit()
        except Exception as exc:  # noqa: BLE001 - never let the timer crash
            self.get_logger().error(f'cache size check failed: {exc}')

    # endregion: handlers

    # region: helpers

    def _require_manager(self) -> ImageManager:
        if self._manager is None:
            raise ImageRepositoryError('node is not configured')
        return self._manager

    def _failure_message(self, operation: str, exc: Exception) -> str:
        """Log a failed request and return the message for the response."""
        if isinstance(exc, NotImplementedError):
            message = 'remote mode not implemented'
        else:
            message = str(exc) or type(exc).__name__
        self.get_logger().error(f'{operation} failed: {message}')
        return message

    def _destroy_entities(self) -> None:
        """Destroy the services and timer and de-initialize the backend."""
        for service in self._image_services:
            self.destroy_service(service)
        self._image_services = []
        if self._cache_timer is not None:
            self._cache_timer.cancel()
            self.destroy_timer(self._cache_timer)
            self._cache_timer = None
        if self._backend is not None:
            try:
                self._backend.shutdown()
            except Exception as exc:  # noqa: BLE001 - keep tearing down
                self.get_logger().error(f'backend shutdown failed: {exc}')

    def _teardown(self) -> None:
        """Release all ROS entities, the backend, the store and the config."""
        self._destroy_entities()
        self._manager = None
        self._store = None
        self._backend = None
        self._config = None

    # endregion: helpers


def main(args: list[str] | None = None) -> None:
    """Run the ``image_repository`` node on a multi-threaded executor."""
    rclpy.init(args=args)
    node = ImageRepository()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
