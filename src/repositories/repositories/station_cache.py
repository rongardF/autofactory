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
"""Typed key/value cache lifecycle node.

``station_cache`` exposes a typed store/fetch/delete service surface over a
pluggable storage backend (local in-memory now; cloud REST later). It owns the
ROS service surface, parameter handling, and the expiry sweep timer, and
delegates every storage operation to a :class:`CacheBackend`. Configuration is
frozen on ``configure``; services exist only while the node is ``active``.
"""

from collections.abc import Iterable
from datetime import datetime
from functools import partial
from typing import SupportsFloat, SupportsIndex, cast

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.service import Service
from rclpy.timer import Timer

from repositories.enums.cache_value_type_enum import CacheValueTypeEnum
from repositories.interfaces.cache_backend import CacheBackend
from repositories.models.cache_entry_dto import CacheEntryDTO, CacheValue
from repositories.models.station_cache_config_dto import StationCacheConfigDTO
from repositories.msg import (
    BooleanCache,
    BytesCache,
    NumberArrayCache,
    NumberCache,
    StringArrayCache,
    StringCache,
)
from repositories.services.cloud_cache_backend import CloudCacheBackend
from repositories.services.local_cache_backend import LocalCacheBackend
from repositories.srv import (
    Delete,
    FetchBoolean,
    FetchBytes,
    FetchNumber,
    FetchNumberArray,
    FetchString,
    FetchStringArray,
    StoreBoolean,
    StoreBytes,
    StoreNumber,
    StoreNumberArray,
    StoreString,
    StoreStringArray,
)
from repositories.utils.time_converter import datetime_to_time_msg, time_msg_to_datetime

# Any ``*Cache`` message type and its class object, used to type the generic
# store/fetch handlers that are shared across all six value types.
CacheMsg = (
    StringCache
    | BooleanCache
    | NumberCache
    | StringArrayCache
    | BytesCache
    | NumberArrayCache
)

# Request/response unions for the generic store and fetch handlers (each is
# bound to a concrete type at service-creation time via functools.partial).
StoreRequest = (
    StoreString.Request
    | StoreBoolean.Request
    | StoreNumber.Request
    | StoreStringArray.Request
    | StoreBytes.Request
    | StoreNumberArray.Request
)
StoreResponse = (
    StoreString.Response
    | StoreBoolean.Response
    | StoreNumber.Response
    | StoreStringArray.Response
    | StoreBytes.Response
    | StoreNumberArray.Response
)
FetchRequest = (
    FetchString.Request
    | FetchBoolean.Request
    | FetchNumber.Request
    | FetchStringArray.Request
    | FetchBytes.Request
    | FetchNumberArray.Request
)
FetchResponse = (
    FetchString.Response
    | FetchBoolean.Response
    | FetchNumber.Response
    | FetchStringArray.Response
    | FetchBytes.Response
    | FetchNumberArray.Response
)


class StationCache(LifecycleNode):
    """Typed key/value cache exposed as a managed lifecycle node."""

    def __init__(self, node_name: str = 'station_cache') -> None:
        """Declare parameters; defer all resource creation to lifecycle transitions.

        Args:
            node_name: The ROS node name. Defaults to ``'station_cache'``.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'local_cache',
            True,
            ParameterDescriptor(
                description='true selects the in-memory LocalCacheBackend; false '
                'selects the CloudCacheBackend (REST, currently stubbed).',
            ),
        )
        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(
                description='Required scope for all keys. Must be a UUID4 string; '
                'configure fails if empty or not a valid UUID4.',
            ),
        )
        self.declare_parameter(
            'cache_service_url',
            '',
            ParameterDescriptor(
                description='Base URL of the REST cache service. Required only when '
                'local_cache is false; ignored in local mode.',
            ),
        )
        self.declare_parameter(
            'expiry_sweep_interval_sec',
            1.0,
            ParameterDescriptor(
                description='Period of the background expiry sweep timer (local '
                'mode). Must be > 0.',
            ),
        )
        self.declare_parameter(
            'request_timeout_sec',
            5.0,
            ParameterDescriptor(
                description='HTTP timeout for cloud backend calls (provisional; '
                'unused until cloud mode is implemented).',
            ),
        )

        # Frozen configuration + backend, populated on configure.
        self._config: StationCacheConfigDTO | None = None
        self._backend: CacheBackend | None = None

        # Live ROS entities, created on activate and destroyed on deactivate.
        # NOTE: avoid the name `_services` because rclpy Node uses it internally.
        self._cache_services: list[Service] = []
        self._sweep_timer: Timer | None = None

    # region: lifecycle

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Read and validate parameters, freeze config, and build the backend.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE on any invalid
            or missing required parameter (logged), leaving the node
            unconfigured. No services are created here.
        """
        try:
            config = StationCacheConfigDTO(
                local_cache=self.get_parameter('local_cache')
                .get_parameter_value()
                .bool_value,
                station_id=self.get_parameter('station_id')
                .get_parameter_value()
                .string_value,
                cache_service_url=self.get_parameter('cache_service_url')
                .get_parameter_value()
                .string_value,
                expiry_sweep_interval_sec=self.get_parameter(
                    'expiry_sweep_interval_sec'
                )
                .get_parameter_value()
                .double_value,
                request_timeout_sec=self.get_parameter('request_timeout_sec')
                .get_parameter_value()
                .double_value,
            )
        except ValidationError as exc:
            self.get_logger().error(f'Invalid configuration: {exc}')
            return TransitionCallbackReturn.FAILURE

        self._config = config

        if config.local_cache:
            self._backend = LocalCacheBackend()
        else:
            self._backend = CloudCacheBackend(
                station_id=config.station_id,
                cache_service_url=config.cache_service_url,
                request_timeout_sec=config.request_timeout_sec,
            )

        mode = 'local' if config.local_cache else 'cloud (stubbed)'
        self.get_logger().info(
            f"Configured station_cache (station_id='{config.station_id}', "
            f'{mode} backend).'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create all services and start the expiry sweep timer (local mode).

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE if the backend
            is not configured or the base activation does not succeed.
        """
        if self._backend is None or self._config is None:
            self.get_logger().error('Cannot activate: node is not configured.')
            return TransitionCallbackReturn.FAILURE

        # Activate managed entities (lifecycle publishers) first; only proceed
        # when the base transition reports success.
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_activate did not succeed; aborting.')
            return TransitionCallbackReturn.FAILURE

        self._create_services()

        # The sweep timer only applies in local mode; the remote service owns
        # expiry in cloud mode.
        if self._config.local_cache:
            self._sweep_timer = self.create_timer(
                self._config.expiry_sweep_interval_sec, self._sweep
            )

        self.get_logger().info('Activated station_cache (services available).')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destroy all services and the sweep timer; retain cache contents.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE if the base
            deactivation does not succeed.
        """
        self._destroy_services()
        if self._sweep_timer is not None:
            self._sweep_timer.cancel()
            self.destroy_timer(self._sweep_timer)
            self._sweep_timer = None

        # Deactivate managed entities (lifecycle publishers); fail the
        # transition if the base transition does not succeed.
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_deactivate did not succeed.')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Deactivated station_cache (cache retained).')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clear the cache, drop the backend, and return to unconfigured.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Cleaned up station_cache.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Tear down all resources on shutdown.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Shut down station_cache.')
        return TransitionCallbackReturn.SUCCESS

    # endregion: lifecycle

    # region: service surface

    def _create_services(self) -> None:
        """Create all 13 node-relative services and track them for teardown."""
        self._cache_services = [
            self.create_service(
                StoreString,
                '~/store_string',
                partial(self._on_store, value_type=CacheValueTypeEnum.STRING),
            ),
            self.create_service(
                StoreBoolean,
                '~/store_boolean',
                partial(self._on_store, value_type=CacheValueTypeEnum.BOOLEAN),
            ),
            self.create_service(
                StoreNumber,
                '~/store_number',
                partial(self._on_store, value_type=CacheValueTypeEnum.NUMBER),
            ),
            self.create_service(
                StoreStringArray,
                '~/store_string_array',
                partial(self._on_store, value_type=CacheValueTypeEnum.STRING_ARRAY),
            ),
            self.create_service(
                StoreBytes,
                '~/store_bytes',
                partial(self._on_store, value_type=CacheValueTypeEnum.BYTES),
            ),
            self.create_service(
                StoreNumberArray,
                '~/store_number_array',
                partial(self._on_store, value_type=CacheValueTypeEnum.NUMBER_ARRAY),
            ),
            self.create_service(
                FetchString,
                '~/fetch_string',
                partial(
                    self._on_fetch,
                    value_type=CacheValueTypeEnum.STRING,
                    msg_factory=StringCache,
                ),
            ),
            self.create_service(
                FetchBoolean,
                '~/fetch_boolean',
                partial(
                    self._on_fetch,
                    value_type=CacheValueTypeEnum.BOOLEAN,
                    msg_factory=BooleanCache,
                ),
            ),
            self.create_service(
                FetchNumber,
                '~/fetch_number',
                partial(
                    self._on_fetch,
                    value_type=CacheValueTypeEnum.NUMBER,
                    msg_factory=NumberCache,
                ),
            ),
            self.create_service(
                FetchStringArray,
                '~/fetch_string_array',
                partial(
                    self._on_fetch,
                    value_type=CacheValueTypeEnum.STRING_ARRAY,
                    msg_factory=StringArrayCache,
                ),
            ),
            self.create_service(
                FetchBytes,
                '~/fetch_bytes',
                partial(
                    self._on_fetch,
                    value_type=CacheValueTypeEnum.BYTES,
                    msg_factory=BytesCache,
                ),
            ),
            self.create_service(
                FetchNumberArray,
                '~/fetch_number_array',
                partial(
                    self._on_fetch,
                    value_type=CacheValueTypeEnum.NUMBER_ARRAY,
                    msg_factory=NumberArrayCache,
                ),
            ),
            self.create_service(Delete, '~/delete', self._on_delete),
        ]

    def _destroy_services(self) -> None:
        """Destroy every live service and clear the tracking list."""
        for service in self._cache_services:
            self.destroy_service(service)
        self._cache_services = []

    # endregion: service surface

    # region: store/fetch/delete handlers

    def _on_store(
        self,
        request: StoreRequest,
        response: StoreResponse,
        value_type: CacheValueTypeEnum,
    ) -> StoreResponse:
        """Store the typed value carried by a ``store_*`` request.

        Args:
            request: The ``Store*`` request whose ``data`` field is a ``*Cache``
                message.
            response: The ``Store*`` response to populate.
            value_type: The logical type of the value being stored.

        Returns:
            The populated response (``success`` + ``error_message``); never
            raises across the boundary.
        """
        if self._backend is None:
            response.success = False
            response.error_message = 'node is not active'
            return response

        data = request.data
        try:
            expire = bool(data.expire_at_valid_till)
            valid_till = time_msg_to_datetime(data.valid_till) if expire else None
            entry = CacheEntryDTO(
                key=data.key,
                value_type=value_type,
                value=self._extract_store_value(value_type, data.value),
                valid_till=valid_till,
                expire_at_valid_till=expire,
                stored_at=self._now(),
            )
            self._backend.store(entry)
        except NotImplementedError:
            self.get_logger().error(f"store '{getattr(data, 'key', '')}': cloud mode")
            response.success = False
            response.error_message = 'cloud mode not implemented'
            return response
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            self.get_logger().error(f'store failed: {exc}')
            response.success = False
            response.error_message = str(exc)
            return response

        response.success = True
        response.error_message = ''
        return response

    def _on_fetch(
        self,
        request: FetchRequest,
        response: FetchResponse,
        value_type: CacheValueTypeEnum,
        msg_factory: type[CacheMsg],
    ) -> FetchResponse:
        """Fetch the typed value for a key, honoring lazy expiry and type match.

        Args:
            request: The ``Fetch*`` request carrying ``key``.
            response: The ``Fetch*`` response to populate.
            value_type: The logical type expected by this fetch service.
            msg_factory: The ``*Cache`` message class for the response payload.

        Returns:
            The populated response (``found`` + ``expired`` + ``data``); never
            raises across the boundary.
        """
        response.found = False
        response.expired = False
        if self._backend is None:
            return response

        try:
            entry, expired = self._backend.fetch(request.key, self._now())
        except NotImplementedError:
            self.get_logger().error(f"fetch '{request.key}': cloud mode")
            return response
        except Exception as exc:  # noqa: BLE001 - convert to miss result
            self.get_logger().error(f'fetch failed: {exc}')
            return response

        # A key may hold an entry of a different type; such a mismatch is
        # reported as a plain miss (found=false, expired=false), regardless of
        # whether that other-typed entry was live or expired.
        if entry is None or entry.value_type is not value_type:
            return response

        if expired:
            response.expired = True
            return response

        response.found = True
        response.data = self._entry_to_msg(entry, msg_factory)
        return response

    def _on_delete(
        self, request: Delete.Request, response: Delete.Response
    ) -> Delete.Response:
        """Delete whatever entry exists for ``key``, regardless of its type.

        Args:
            request: The ``Delete`` request carrying ``key``.
            response: The ``Delete`` response to populate.

        Returns:
            The populated response (``success`` + ``deleted`` + ``error_message``);
            never raises across the boundary.
        """
        response.success = False
        response.deleted = False
        response.error_message = ''
        if self._backend is None:
            response.error_message = 'node is not active'
            return response

        try:
            response.deleted = self._backend.delete(request.key)
        except NotImplementedError:
            self.get_logger().error(f"delete '{request.key}': cloud mode")
            response.error_message = 'cloud mode not implemented'
            return response
        except Exception as exc:  # noqa: BLE001 - convert to failure result
            self.get_logger().error(f'delete failed: {exc}')
            response.error_message = str(exc)
            return response

        response.success = True
        return response

    def _sweep(self) -> None:
        """Timer callback: proactively purge expired entries (local mode)."""
        if self._backend is None:
            return
        try:
            self._backend.sweep_expired(self._now())
        except Exception as exc:  # noqa: BLE001 - never let the timer crash
            self.get_logger().error(f'expiry sweep failed: {exc}')

    # endregion: store/fetch/delete handlers

    # region: helpers

    def _extract_store_value(
        self, value_type: CacheValueTypeEnum, raw: object
    ) -> CacheValue:
        """Normalize a raw ROS message value into the DTO payload type.

        The incoming ``*Cache`` message field is dynamically typed, so each
        branch narrows ``raw`` to the concrete shape guaranteed by
        ``value_type`` before coercing it.

        Args:
            value_type: The logical type of the value.
            raw: The ``value`` field from the incoming ``*Cache`` message.

        Returns:
            CacheValue: The value coerced to the Python type matching
            ``value_type``.
        """
        if value_type is CacheValueTypeEnum.STRING:
            return str(raw)
        if value_type is CacheValueTypeEnum.BOOLEAN:
            return bool(raw)
        if value_type is CacheValueTypeEnum.NUMBER:
            return float(cast(SupportsFloat, raw))
        if value_type is CacheValueTypeEnum.STRING_ARRAY:
            return [str(item) for item in cast(Iterable[object], raw)]
        if value_type is CacheValueTypeEnum.BYTES:
            return bytes(cast(Iterable[SupportsIndex], raw))
        return [float(item) for item in cast(Iterable[SupportsFloat], raw)]

    def _entry_to_msg(
        self, entry: CacheEntryDTO, msg_factory: type[CacheMsg]
    ) -> CacheMsg:
        """Build the ``*Cache`` response message from a stored entry.

        Args:
            entry: The live entry to serialize.
            msg_factory: The ``*Cache`` message class to instantiate.

        Returns:
            The populated ``*Cache`` message.
        """
        msg = msg_factory()
        msg.key = entry.key
        msg.expire_at_valid_till = entry.expire_at_valid_till
        if entry.valid_till is not None:
            msg.valid_till = datetime_to_time_msg(entry.valid_till)
        msg.value = entry.value
        return msg

    def _now(self) -> datetime:
        """Return the current instant from the node clock (sim-time aware).

        Returns:
            datetime: The current time as a timezone-aware UTC ``datetime``.
        """
        return time_msg_to_datetime(self.get_clock().now().to_msg())

    def _teardown(self) -> None:
        """Release services, the sweep timer, cache contents, and the backend."""
        self._destroy_services()
        if self._sweep_timer is not None:
            self._sweep_timer.cancel()
            self.destroy_timer(self._sweep_timer)
            self._sweep_timer = None
        if self._backend is not None:
            try:
                self._backend.clear()
            except NotImplementedError:
                pass
            self._backend = None
        self._config = None

    # endregion: helpers


def main(args: list[str] | None = None) -> None:
    """Spin the station cache lifecycle node until shutdown.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = StationCache()
    executor = MultiThreadedExecutor(num_threads=3)
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
