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
"""Material repository lifecycle node.

``material_repository`` serves read-only material data, referenced by product
number and revision, to other nodes (for example endtools). In local mode the
materials file is parsed and validated on ``configure``; in cloud mode the
backend is a stub. The ``~/get_material_data`` service exists only while the
node is ``active``.
"""

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.service import Service

from repositories.exceptions.material_repository.material_not_found_error import (
    MaterialNotFoundError,
)
from repositories.interfaces.material_repository.material_backend import (
    MaterialBackend,
)
from repositories.models.material_repository.material_repository_config_dto import (
    MaterialRepositoryConfigDTO,
)
from repositories.msg import MaterialRepositoryMaterialData
from repositories.services.material_repository.cloud_material_backend import (
    CloudMaterialBackend,
)
from repositories.services.material_repository.local_material_backend import (
    LocalMaterialBackend,
)
from repositories.services.material_repository.material_service import MaterialService
from repositories.srv import MaterialRepositoryGetMaterialData


class MaterialRepository(LifecycleNode):
    """Read-only material data exposed as a lifecycle node."""

    def __init__(self, node_name: str = 'material_repository') -> None:
        """Declare parameters; defer all resource creation to lifecycle transitions.

        Args:
            node_name: The ROS node name. Defaults to ``'material_repository'``.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(
                description='Required station scope. Must be a UUID4 string; '
                'configure fails otherwise.',
            ),
        )
        self.declare_parameter(
            'local_storage',
            True,
            ParameterDescriptor(
                description='true serves materials from the local materials file '
                'and never uses the network; false uses the cloud material '
                'service (currently stubbed).',
            ),
        )
        self.declare_parameter(
            'material_service_url',
            '',
            ParameterDescriptor(
                description='Base URL of the material REST service. Required '
                'only when local_storage is false.',
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
        self.declare_parameter(
            'materials_file_path',
            '/tmp/station/materials.json',
            ParameterDescriptor(
                description='Absolute path of the JSON file listing the '
                'materials; parsed and validated on configure in local mode.',
            ),
        )

        # Frozen configuration, data source and internal service; set on configure.
        self._config: MaterialRepositoryConfigDTO | None = None
        self._backend: MaterialBackend | None = None
        self._material_service: MaterialService | None = None

        # Live ROS entity, created on activate and destroyed on deactivate.
        self._get_material_data_service: Service | None = None

    # region: lifecycle

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate parameters, freeze config and load the material source.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE on any invalid
            parameter or, in local mode, if the materials file cannot be loaded.
        """
        self._teardown()
        try:
            config = MaterialRepositoryConfigDTO(
                station_id=self.get_parameter('station_id')
                .get_parameter_value()
                .string_value,
                local_storage=self.get_parameter('local_storage')
                .get_parameter_value()
                .bool_value,
                material_service_url=self.get_parameter('material_service_url')
                .get_parameter_value()
                .string_value,
                request_timeout_sec=self.get_parameter('request_timeout_sec')
                .get_parameter_value()
                .double_value,
                materials_file_path=self.get_parameter('materials_file_path')
                .get_parameter_value()
                .string_value,
            )
        except ValidationError as error:
            self.get_logger().error(f'Invalid configuration: {error}')
            return TransitionCallbackReturn.FAILURE

        backend: MaterialBackend
        if config.local_storage:
            backend = LocalMaterialBackend(config.materials_file_path)
        else:
            backend = CloudMaterialBackend(
                station_id=config.station_id,
                material_service_url=config.material_service_url,
                request_timeout_sec=config.request_timeout_sec,
            )
        try:
            material_count = backend.load()
        except ValueError as error:
            self.get_logger().error(f'Failed to load materials: {error}')
            return TransitionCallbackReturn.FAILURE

        self._config = config
        self._backend = backend
        # Created here, initialized only on activate.
        self._material_service = MaterialService(backend)

        if config.local_storage:
            detail = (
                f"local mode, {material_count} materials loaded from "
                f"'{config.materials_file_path}'"
            )
        else:
            detail = 'cloud mode (stubbed)'
        self.get_logger().info(
            f"Configured material_repository (station_id='{config.station_id}', "
            f'{detail}).'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create the lookup service and initialize the internal service.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE if the node is
            not configured or the base activation does not succeed.
        """
        if self._config is None or self._material_service is None:
            self.get_logger().error('Cannot activate: node is not configured.')
            return TransitionCallbackReturn.FAILURE

        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_activate did not succeed; aborting.')
            return TransitionCallbackReturn.FAILURE

        try:
            self._get_material_data_service = self.create_service(
                MaterialRepositoryGetMaterialData,
                '~/get_material_data',
                self._on_get_material_data,
            )
        except Exception as error:  # noqa: BLE001 - activation must report failure
            self._destroy_service()
            self.get_logger().error(
                f'Failed to create get_material_data service: {error}'
            )
            return TransitionCallbackReturn.FAILURE

        self._material_service.initialize()
        self.get_logger().info(
            'Activated material_repository (service: ~/get_material_data).'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """De-initialize the internal service and destroy the lookup service.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE if the base
            deactivation does not succeed.
        """
        if self._material_service is not None:
            self._material_service.deinitialize()
        self._destroy_service()

        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Base on_deactivate did not succeed.')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Deactivated material_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the config, the material cache and the internal service.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Cleaned up material_repository.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release all resources on shutdown.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Shut down material_repository.')
        return TransitionCallbackReturn.SUCCESS

    # endregion: lifecycle

    # region: service handler

    def _on_get_material_data(
        self,
        request: MaterialRepositoryGetMaterialData.Request,
        response: MaterialRepositoryGetMaterialData.Response,
    ) -> MaterialRepositoryGetMaterialData.Response:
        """Return the material matching the requested number and revision.

        The match is an exact, case-insensitive match of the (number, revision)
        pair.

        Args:
            request: Request carrying the product number and revision.
            response: Response to populate.

        Returns:
            The populated response; never raises across the boundary.
        """
        response.material_data = MaterialRepositoryMaterialData()
        requested = (
            f"product_number='{request.product_number}', "
            f"product_revision='{request.product_revision}'"
        )
        try:
            service = self._material_service
            if service is None:
                raise RuntimeError('material repository is not configured')
            material = service.get_material(
                request.product_number, request.product_revision
            )
        except MaterialNotFoundError:
            return self._fail(response, 'material not found', requested)
        except NotImplementedError:
            return self._fail(response, 'cloud mode not implemented', requested)
        except Exception as error:  # noqa: BLE001 - convert to failure result
            return self._fail(
                response, f'failed to get material data: {error}', requested
            )

        response.material_data = material.to_msg()
        response.success = True
        response.error_message = ''
        self.get_logger().info(f'get_material_data: found {requested}.')
        return response

    def _fail(
        self,
        response: MaterialRepositoryGetMaterialData.Response,
        message: str,
        requested: str,
    ) -> MaterialRepositoryGetMaterialData.Response:
        """Log an error and fill the response as a failure."""
        self.get_logger().error(f'get_material_data failed ({requested}): {message}')
        response.success = False
        response.error_message = message
        return response

    # endregion: service handler

    # region: helpers

    def _destroy_service(self) -> None:
        """Destroy the lookup service if it exists."""
        if self._get_material_data_service is not None:
            self.destroy_service(self._get_material_data_service)
            self._get_material_data_service = None

    def _teardown(self) -> None:
        """Release the service interface, internal service, cache and config."""
        self._destroy_service()
        if self._material_service is not None:
            self._material_service.deinitialize()
        self._material_service = None
        self._backend = None
        self._config = None

    # endregion: helpers


def main(args: list[str] | None = None) -> None:
    """Spin the material repository lifecycle node until shutdown.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = MaterialRepository()
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
