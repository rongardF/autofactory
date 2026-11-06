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
"""``spawn_product`` / ``remove_product`` services of a simulated workplace."""

import os
import threading

import xacro
from pydantic import ValidationError
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.service import Service
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException  # type: ignore[attr-defined]

from repositories.enums.object_model_storage.model_type_enum import ModelTypeEnum
from repositories.srv import ObjectModelStorageGetModel, ObjectModelStorageGetModelsInfo
from station_workplace.exceptions.spawn_error import SpawnError
from station_workplace.exceptions.world_manager_error import WorldManagerError
from station_workplace.interface.simulation_hooks import SimulationHooks
from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.sdf_model_dto import SdfModelDTO
from station_workplace.models.sim_model_metadata_dto import SimModelMetadataDTO
from station_workplace.models.simulated_product_dto import SimulatedProductDTO
from station_workplace.services.collision_checker import CollisionChecker
from station_workplace.services.gazebo_world_manager import GazeboWorldManager
from station_workplace.srv import RemoveProduct, SpawnProduct
from station_workplace.utils.async_service_caller import call_service
from station_workplace.utils.sdf_parser import find_link_pose, parse_sdf_model
from station_workplace.utils.transformations import (
    matrix_to_pose,
    pose6_to_matrix,
    transform_to_matrix,
)

_WORLD_FRAME = 'world'
_MODEL_FILE = 'model.sdf.xacro'


class SimulatedProductSpawner:
    """Spawn a product with its carrier into Gazebo and remove them again.

    Services (created only while active):

    * ``~/spawn_product`` (``station_workplace/SpawnProduct``);
    * ``~/remove_product`` (``station_workplace/RemoveProduct``).

    The services never touch the workplace process state. Both run in one
    mutually exclusive callback group and additionally share a lock.
    """

    def __init__(
        self,
        node: Node,
        config: CommonNodeConfigDTO,
        world: GazeboWorldManager,
        hooks: SimulationHooks,
        io_group: CallbackGroup,
    ) -> None:
        """Store the collaborators; interfaces are created by :meth:`activate`."""
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._sim = config.simulation
        self._world = world
        self._hooks = hooks
        self._io_group = io_group
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._checker = CollisionChecker(self._sim.penetration_tolerance)
        self._lock = threading.Lock()
        self._never_cancelled = threading.Event()
        self._models_info_client: Client | None = None
        self._model_client: Client | None = None
        self._spawn_service: Service | None = None
        self._remove_service: Service | None = None
        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None

    # region lifecycle
    def activate(self) -> None:
        """Create the clients, the TF listener, the hook interfaces and the services."""
        timeout = self._config.service_call_timeout_sec
        self._world.activate(self._sim, timeout, self._io_group)
        self._hooks.activate(self._node)
        self._models_info_client = self._node.create_client(
            ObjectModelStorageGetModelsInfo, self._config.get_models_info_service,
            callback_group=self._io_group)
        self._model_client = self._node.create_client(
            ObjectModelStorageGetModel, self._config.get_model_service,
            callback_group=self._io_group)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)
        self._spawn_service = self._node.create_service(
            SpawnProduct, '~/spawn_product', self._on_spawn_product,
            callback_group=self._service_group)
        self._remove_service = self._node.create_service(
            RemoveProduct, '~/remove_product', self._on_remove_product,
            callback_group=self._service_group)

    def deactivate(self) -> None:
        """Destroy all interfaces; spawned models stay in the world and in the record."""
        for name in ('_spawn_service', '_remove_service'):
            service = getattr(self, name)
            if service is not None:
                self._node.destroy_service(service)
                setattr(self, name, None)
        for name in ('_models_info_client', '_model_client'):
            client = getattr(self, name)
            if client is not None:
                self._node.destroy_client(client)
                setattr(self, name, None)
        if self._tf_listener is not None:
            self._tf_listener.unregister()
        self._tf_listener = None
        self._tf_buffer = None
        self._hooks.deactivate()
        self._world.deactivate()
    # endregion lifecycle

    # region service callbacks
    def _on_spawn_product(
        self, request: SpawnProduct.Request, response: SpawnProduct.Response,
    ) -> SpawnProduct.Response:
        with self._lock:
            try:
                self._spawn(request)
            except Exception as exc:  # noqa: BLE001 - converted to a result at the boundary
                self._logger.error(f'spawn_product failed: {exc}')
                response.success = False
                response.error_message = str(exc)
                return response
        response.success = True
        response.error_message = ''
        return response

    def _on_remove_product(
        self, request: RemoveProduct.Request, response: RemoveProduct.Response,
    ) -> RemoveProduct.Response:
        with self._lock:
            try:
                self._remove_all()
            except Exception as exc:  # noqa: BLE001 - converted to a result at the boundary
                self._logger.error(f'remove_product failed: {exc}')
                response.success = False
                response.error_message = str(exc)
                return response
        response.success = True
        response.error_message = ''
        return response
    # endregion service callbacks

    # region spawn
    def _spawn(self, request: SpawnProduct.Request) -> None:
        if self._world.spawned_models:
            raise SpawnError(
                f'product already spawned ({", ".join(self._world.spawned_models)}); '
                'call remove_product first')
        product = self._validate(request)
        carrier_extra = self._hooks.carrier_mappings(product.product)

        product_info = self._get_single_sdf_model_info(product.assembly_model_id, 'product')
        try:
            metadata = SimModelMetadataDTO.model_validate_json(product_info.metadata_json or '')
        except ValidationError as exc:
            raise SpawnError(
                f"invalid metadata_json of product model '{product.assembly_model_id}': "
                f'{exc}') from exc
        product_dir = self._get_model_dir(product.assembly_model_id, 'product')
        self._get_single_sdf_model_info(metadata.carrier_model_id, 'carrier')
        carrier_dir = self._get_model_dir(metadata.carrier_model_id, 'carrier')

        carrier_sdf = self._expand(carrier_dir, 'carrier', {
            'model_name': self._sim.carrier_model_name,
            'station_model_name': self._sim.station_model_name,
            'station_link': self._sim.station_link,
            **carrier_extra,
        })
        product_sdf = self._expand(product_dir, 'product', {
            'model_name': self._sim.product_model_name,
            'carrier_model_name': self._sim.carrier_model_name,
            'carrier_link': self._sim.carrier_link,
        })
        carrier_model = self._parse(carrier_sdf, carrier_dir, 'carrier')
        product_model = self._parse(product_sdf, product_dir, 'product')

        try:
            carrier_link_pose = pose6_to_matrix(
                find_link_pose(carrier_model, self._sim.carrier_link))
        except ValueError as exc:
            raise SpawnError(f'carrier model: {exc}') from exc
        product_in_carrier = carrier_link_pose @ pose6_to_matrix(product.pose)
        self._checker.check(carrier_model, product_model, product_in_carrier)

        carrier_in_world = self._lookup_station_link()
        try:
            self._world.spawn_model(
                self._sim.carrier_model_name, carrier_sdf, matrix_to_pose(carrier_in_world))
            self._world.spawn_model(
                self._sim.product_model_name, product_sdf,
                matrix_to_pose(carrier_in_world @ product_in_carrier))
            self._hooks.after_spawn(product.product)
        except Exception as exc:
            rollback_error = self._rollback()
            if rollback_error:
                raise SpawnError(f'{exc}; rollback failed: {rollback_error}') from exc
            raise
        self._logger.info(
            f"Spawned product '{product.product.serial_number}' "
            f"(model '{product.assembly_model_id}', carrier '{metadata.carrier_model_id}')")

    @staticmethod
    def _validate(request: SpawnProduct.Request) -> SimulatedProductDTO:
        try:
            return SimulatedProductDTO(
                product=ProductDataDTO(
                    product_number=request.product_number,
                    product_revision=request.product_revision,
                    serial_number=request.serial_number,
                ),
                assembly_model_id=request.assembly_model_id,
                pose=request.pose,  # type: ignore
            )
        except ValidationError as exc:
            raise SpawnError(f'invalid request: {exc}') from exc

    def _get_single_sdf_model_info(self, model_id: str, description: str):
        request = ObjectModelStorageGetModelsInfo.Request(model_id=model_id)
        response = call_service(
            self._require(self._models_info_client), request,
            self._config.service_call_timeout_sec, self._never_cancelled, SpawnError,
            f'get {description} model info')
        if not response.success:
            raise SpawnError(
                f"get {description} model info '{model_id}' failed: {response.error_message}")
        if len(response.models) != 1:
            raise SpawnError(
                f"expected 1 {description} model '{model_id}', got {len(response.models)}")
        info = response.models[0]
        if info.model_type != ModelTypeEnum.SDF.value:
            raise SpawnError(
                f"{description} model '{model_id}' has type '{info.model_type}', expected 'sdf'")
        return info

    def _get_model_dir(self, model_id: str, description: str) -> str:
        request = ObjectModelStorageGetModel.Request(model_id=model_id)
        response = call_service(
            self._require(self._model_client), request,
            self._config.service_call_timeout_sec, self._never_cancelled, SpawnError,
            f'get {description} model')
        if not response.success:
            raise SpawnError(
                f"get {description} model '{model_id}' failed: {response.error_message}")
        if not os.path.isdir(response.local_path):
            raise SpawnError(f"{description} model folder '{response.local_path}' does not exist")
        return response.local_path

    @staticmethod
    def _expand(model_dir: str, description: str, mappings: dict[str, str]) -> str:
        path = os.path.join(model_dir, _MODEL_FILE)
        if not os.path.isfile(path):
            raise SpawnError(f"{description} model file '{path}' does not exist")
        try:
            return xacro.process_file(path, mappings=mappings).toxml()  # type: ignore
        except Exception as exc:  # noqa: BLE001 - xacro raises various exception types
            raise SpawnError(f"failed to expand {description} model '{path}': {exc}") from exc

    @staticmethod
    def _parse(sdf: str, model_dir: str, description: str) -> SdfModelDTO:
        try:
            return parse_sdf_model(sdf, model_dir)
        except ValueError as exc:
            raise SpawnError(f'{description} model: {exc}') from exc

    def _lookup_station_link(self):
        if self._tf_buffer is None:
            raise SpawnError('TF buffer is not active')
        try:
            transform = self._tf_buffer.lookup_transform(
                _WORLD_FRAME, self._sim.station_link, Time(),
                timeout=Duration(seconds=self._config.service_call_timeout_sec))
        except TransformException as exc:
            raise SpawnError(
                f"failed to look up '{self._sim.station_link}' in '{_WORLD_FRAME}': {exc}") from exc
        return transform_to_matrix(transform.transform)

    def _rollback(self) -> str:
        """Remove the models spawned so far; return the error message or ``''``."""
        try:
            self._remove_all()
        except Exception as exc:  # noqa: BLE001 - reported together with the original error
            self._logger.error(f'Rollback failed: {exc}')
            return str(exc)
        return ''
    # endregion spawn

    # region remove
    def _remove_all(self) -> None:
        """Remove the spawned models in reverse spawn order (product before carrier).

        Stops at the first failure; models not removed stay in the record.
        """
        for name in reversed(self._world.spawned_models):
            try:
                self._world.remove_model(name)
            except WorldManagerError as exc:
                raise WorldManagerError(f"failed to remove model '{name}': {exc}") from exc
    # endregion remove

    @staticmethod
    def _require(client: Client | None) -> Client:
        if client is None:
            raise SpawnError('simulated product spawner is not active')
        return client
