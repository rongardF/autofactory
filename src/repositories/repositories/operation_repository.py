#!/usr/bin/env python3
# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS WARRANTIES ARE DISCLAIMED.
"""Lifecycle node serving validated operations from a configured backend."""

from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.service import Service

from repositories.interfaces.operation_repository.operation_backend import (
    OperationBackend,
)
from repositories.models.operation_repository.operation_repository_config_dto import (
    OperationRepositoryConfigDTO,
)
from repositories.services.operation_repository.cloud_operation_backend import (
    CloudOperationBackend,
)
from repositories.services.operation_repository.local_operation_backend import (
    LocalOperationBackend,
)
from repositories.srv import OperationRepositoryGetOperation
from repositories.utils.uuid_validator import parse_uuid4


class OperationRepository(LifecycleNode):
    """Provide one operation lookup service with local and cloud backends."""

    def __init__(self, node_name: str = 'operation_repository') -> None:
        super().__init__(node_name)
        self.declare_parameter(
            'local_repository',
            True,
            ParameterDescriptor(
                description='Read the local assembly file when true; use the '
                'cloud operation repository when false.',
            ),
        )
        self.declare_parameter(
            'assembly_file_path',
            '/tmp/station/mock_assembly_file.json',
            ParameterDescriptor(
                description='Absolute path to the assembly JSON file in local mode.',
            ),
        )
        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(
                description='Required station UUID4 in cloud mode.',
            ),
        )
        self.declare_parameter(
            'operation_repository_url',
            '',
            ParameterDescriptor(
                description='REST base URL; required in cloud mode.',
            ),
        )
        self.declare_parameter(
            'product_origin_frame_id',
            'product_origin',
            ParameterDescriptor(
                description='Frame id assigned to every converted pose.',
            ),
        )
        self._config: OperationRepositoryConfigDTO | None = None
        self._backend: OperationBackend | None = None
        self._get_operation_service: Service | None = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate parameters and prepare the selected operation source."""
        self._config = None
        self._backend = None
        try:
            config = OperationRepositoryConfigDTO(
                local_repository=self.get_parameter('local_repository')
                .get_parameter_value()
                .bool_value,
                assembly_file_path=self.get_parameter('assembly_file_path')
                .get_parameter_value()
                .string_value,
                station_id=self.get_parameter('station_id')
                .get_parameter_value()
                .string_value,
                operation_repository_url=self.get_parameter('operation_repository_url')
                .get_parameter_value()
                .string_value,
                product_origin_frame_id=self.get_parameter('product_origin_frame_id')
                .get_parameter_value()
                .string_value,
            )
        except ValidationError as error:
            self.get_logger().error(f'Invalid configuration: {error}')
            return TransitionCallbackReturn.FAILURE

        try:
            if config.local_repository:
                local_backend = LocalOperationBackend(
                    config.assembly_file_path
                )
                backend: OperationBackend = local_backend
                operation_count = local_backend.operation_count
            else:
                backend = CloudOperationBackend(
                    config.station_id,
                    config.operation_repository_url,
                )
                operation_count = None
        except Exception as error:
            self.get_logger().error(f'Failed to configure operation backend: {error}')
            self._config = None
            self._backend = None
            return TransitionCallbackReturn.FAILURE

        self._config = config
        self._backend = backend
        mode = 'local' if config.local_repository else 'cloud (stubbed)'
        detail = (
            f', {operation_count} operations loaded'
            if operation_count is not None
            else ''
        )
        self.get_logger().info(
            f'Configured operation_repository in {mode} mode{detail}.'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create the lookup service for the active lifecycle state."""
        if self._config is None or self._backend is None:
            self.get_logger().error('Cannot activate: node is not configured.')
            return TransitionCallbackReturn.FAILURE
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_activate did not succeed.')
            return TransitionCallbackReturn.FAILURE
        try:
            self._get_operation_service = self.create_service(
                OperationRepositoryGetOperation,
                '~/get_operation',
                self._on_get_operation,
            )
        except Exception as error:
            self._destroy_service()
            self.get_logger().error(f'Failed to create get_operation service: {error}')
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Activated operation_repository (service: ~/get_operation).')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destroy the lookup service before leaving the active state."""
        self._destroy_service()
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_deactivate did not succeed.')
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Deactivated operation_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the service, backend and configuration."""
        self._teardown()
        self.get_logger().info('Cleaned up operation_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release all node resources during shutdown."""
        self._teardown()
        self.get_logger().info('Shut down operation_repository.')
        return TransitionCallbackReturn.SUCCESS

    def _on_get_operation(
        self,
        request: OperationRepositoryGetOperation.Request,
        response: OperationRepositoryGetOperation.Response,
    ) -> OperationRepositoryGetOperation.Response:
        """Validate request identifiers, fetch one operation and convert it."""
        response.operation = []
        backend = self._backend
        config = self._config
        try:
            operation_id = parse_uuid4(request.operation_id, 'operation_id')
            assembly_id = parse_uuid4(request.assembly_id, 'assembly_id')
            if backend is None or config is None:
                raise RuntimeError('operation_repository is not configured')
            operation = backend.get_operation(operation_id, assembly_id)
            operation_message = operation.to_msg(config.product_origin_frame_id)
        except Exception as error:
            message = f'failed to get operation: {error}'
            self.get_logger().error(message)
            response.success = False
            response.error_message = message
            return response

        response.operation = [operation_message]
        response.success = True
        response.error_message = ''
        self.get_logger().info(
            f'get_operation: returned operation {operation_id} '
            f'from assembly {assembly_id}.'
        )
        return response

    def _destroy_service(self) -> None:
        """Destroy the lookup service if it exists."""
        if self._get_operation_service is not None:
            self.destroy_service(self._get_operation_service)
            self._get_operation_service = None

    def _teardown(self) -> None:
        """Release the live service and configured backend state."""
        self._destroy_service()
        self._backend = None
        self._config = None


def main(args: list[str] | None = None) -> None:
    """Spin the operation repository lifecycle node until shutdown."""
    rclpy.init(args=args)
    node = OperationRepository()
    executor = MultiThreadedExecutor(num_threads=2)
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
