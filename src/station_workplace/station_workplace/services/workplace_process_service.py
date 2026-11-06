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
"""Injection and ejection of the product in the workplace."""

import threading
from collections.abc import Callable
from pathlib import Path

from pydantic import ValidationError
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node

from production_manager.srv import GetProductionData
from repositories.enums.object_model_storage.model_type_enum import ModelTypeEnum
from repositories.srv import ObjectModelStorageGetModel, ObjectModelStorageGetModelsInfo
from station_workplace.enums.constants import CARRIER_OBJECT_ID, PRODUCT_OBJECT_ID, SAFETY_OBJECT_ID
from station_workplace.exceptions.injection_error import InjectionError
from station_workplace.exceptions.process_cancelled_error import ProcessCancelledError
from station_workplace.interface.planning_scene_manager import PlanningSceneManager
from station_workplace.interface.product_data_reader import ProductDataReader
from station_workplace.models.model_metadata_dto import ModelMetadataDTO
from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.workplace_state_dto import WorkplaceStateDTO
from station_workplace.services.process_runner import ProcessRunner
from station_workplace.services.product_origin_tf_publisher import ProductOriginTfPublisher
from station_workplace.services.workplace_state_publisher import WorkplaceStatePublisher
from station_workplace.utils.async_service_caller import call_service
from station_workplace.utils.transformations import pose6_to_pose


class WorkplaceProcessService:
    """Shared injection/ejection logic of all workplace nodes.

    Both processes run on :class:`ProcessRunner` worker threads. Every
    planning-scene change and publication runs inside the runner's side-effect
    guard, so nothing happens once a lifecycle transition disabled the runner.
    """

    def __init__(
        self,
        node: Node,
        config: CommonNodeConfigDTO,
        scene: PlanningSceneManager,
        state_publisher: WorkplaceStatePublisher,
        tf_publisher: ProductOriginTfPublisher,
        reader: ProductDataReader,
        runner: ProcessRunner,
        callback_group: CallbackGroup,
        on_system_failure: Callable[[str], None],
    ) -> None:
        """Store collaborators; service clients are created in :meth:`activate`.

        :param node: Node used to create clients and access the logger.
        :param config: Validated node configuration.
        :param scene: Planning-scene manager.
        :param state_publisher: ``~/workplace`` publisher.
        :param tf_publisher: ``estimated_product_origin`` TF publisher.
        :param reader: Node-specific product data reader.
        :param runner: Process runner providing the side-effect guard.
        :param callback_group: Callback group of the service clients.
        :param on_system_failure: Called (from the worker) on a system failure.
        """
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._scene = scene
        self._state = state_publisher
        self._tf = tf_publisher
        self._reader = reader
        self._runner = runner
        self._callback_group = callback_group
        self._on_system_failure = on_system_failure
        self._production_client: Client | None = None
        self._models_info_client: Client | None = None
        self._model_client: Client | None = None

    # region lifecycle
    def activate(self) -> None:
        """Create the production manager and object model storage clients."""
        self._production_client = self._node.create_client(
            GetProductionData, self._config.get_production_data_service,
            callback_group=self._callback_group)
        self._models_info_client = self._node.create_client(
            ObjectModelStorageGetModelsInfo, self._config.get_models_info_service,
            callback_group=self._callback_group)
        self._model_client = self._node.create_client(
            ObjectModelStorageGetModel, self._config.get_model_service,
            callback_group=self._callback_group)

    def deactivate(self) -> None:
        """Destroy the service clients."""
        for name in ('_production_client', '_models_info_client', '_model_client'):
            client = getattr(self, name)
            if client is not None:
                self._node.destroy_client(client)
                setattr(self, name, None)
    # endregion lifecycle

    # region injection
    def inject(self, cancel: threading.Event, epoch: int) -> None:
        """Identify the product and attach its models (worker thread)."""
        try:
            with self._runner.side_effect_guard(epoch):
                self._scene.attach_safety_object(cancel)
        except ProcessCancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - any failure here is a system failure
            self._fail_system(f'attaching the safety object failed: {exc}')
            return

        try:
            self._inject_product(cancel, epoch)
        except ProcessCancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - every step 2-10 failure is an injection error
            self._logger.error(f'Injection error: {exc}')
            self._recover_from_injection_error(cancel, epoch)

    def _inject_product(self, cancel: threading.Event, epoch: int) -> None:
        """Run injection steps 2-11; raises on any failure."""
        product = self._reader.read(cancel)
        self._logger.info(f'Product data: {product}')
        self._check(cancel)

        assembly_model_id = self._get_assembly_model_id(product, cancel)
        self._check(cancel)

        product_info = self._get_single_stl_model_info(assembly_model_id, 'product', cancel)
        try:
            metadata = ModelMetadataDTO.model_validate_json(product_info.metadata_json or '')
        except ValidationError as exc:
            raise InjectionError(
                f"invalid metadata_json of product model '{assembly_model_id}': {exc}") from exc
        self._check(cancel)

        product_path = self._get_model_path(assembly_model_id, 'product', cancel)
        self._check(cancel)
        self._get_single_stl_model_info(metadata.carrier_model_id, 'carrier', cancel)
        self._check(cancel)
        carrier_path = self._get_model_path(metadata.carrier_model_id, 'carrier', cancel)
        self._check(cancel)

        pose6 = metadata.estimated_product_origin_pose
        with self._runner.side_effect_guard(epoch):
            self._scene.attach_product_and_carrier(
                product_path, carrier_path, pose6_to_pose(pose6), cancel)
        with self._runner.side_effect_guard(epoch):
            self._tf.publish_pose(pose6)
        with self._runner.side_effect_guard(epoch):
            self._state.publish(WorkplaceStateDTO(in_place=True, product_data=product))
        self._logger.info('Injection completed')

    def _get_assembly_model_id(self, product: ProductDataDTO, cancel: threading.Event) -> str:
        request = GetProductionData.Request(
            product_number=product.product_number,
            product_revision=product.product_revision,
            serial_number=product.serial_number,
        )
        response = call_service(
            self._require(self._production_client), request,
            self._config.service_call_timeout_sec, cancel, InjectionError,
            'get production data')
        if not response.success:
            raise InjectionError(f'get production data failed: {response.error_message}')
        if not response.assembly_model_id:
            raise InjectionError('get production data returned an empty assembly_model_id')
        self._logger.info(
            f'Production data: assembly_id={response.assembly_id}, '
            f'operation_id={response.operation_id}, '
            f'assembly_model_id={response.assembly_model_id}')
        return response.assembly_model_id

    def _get_single_stl_model_info(self, model_id: str, description: str, cancel: threading.Event):
        request = ObjectModelStorageGetModelsInfo.Request(model_id=model_id)
        response = call_service(
            self._require(self._models_info_client), request,
            self._config.service_call_timeout_sec, cancel, InjectionError,
            f'get {description} model info')
        if not response.success:
            raise InjectionError(
                f"get {description} model info '{model_id}' failed: {response.error_message}")
        if len(response.models) != 1:
            raise InjectionError(
                f"expected 1 {description} model '{model_id}', got {len(response.models)}")
        info = response.models[0]
        if info.model_type != ModelTypeEnum.STL.value:
            raise InjectionError(
                f"{description} model '{model_id}' has type '{info.model_type}', expected 'stl'")
        return info

    def _get_model_path(self, model_id: str, description: str, cancel: threading.Event) -> Path:
        request = ObjectModelStorageGetModel.Request(model_id=model_id)
        response = call_service(
            self._require(self._model_client), request,
            self._config.service_call_timeout_sec, cancel, InjectionError,
            f'get {description} model')
        if not response.success:
            raise InjectionError(
                f"get {description} model '{model_id}' failed: {response.error_message}")
        path = Path(response.local_path)
        if not path.is_file():
            raise InjectionError(f"{description} model file '{path}' does not exist")
        return path

    def _recover_from_injection_error(self, cancel: threading.Event, epoch: int) -> None:
        """Fall back to the safety object and the UNKNOWN state."""
        try:
            with self._runner.side_effect_guard(epoch):
                present = self._scene.get_present_object_ids(cancel)
                self._scene.remove_objects([PRODUCT_OBJECT_ID, CARRIER_OBJECT_ID], cancel)
                if SAFETY_OBJECT_ID not in present:
                    self._scene.attach_safety_object(cancel)
        except ProcessCancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - the safety object could not be ensured
            self._fail_system(f'restoring the safety object failed: {exc}')
            return

        try:
            with self._runner.side_effect_guard(epoch):
                self._tf.reset_to_identity()
            with self._runner.side_effect_guard(epoch):
                self._state.publish(
                    WorkplaceStateDTO(in_place=True, product_data=ProductDataDTO.unknown()))
        except ProcessCancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - log; the ejection will clean up
            self._logger.error(f'Publishing the UNKNOWN state failed: {exc}')
    # endregion injection

    # region ejection
    def eject(self, cancel: threading.Event, epoch: int) -> None:
        """Remove all workplace objects and publish the empty state (worker thread)."""
        try:
            with self._runner.side_effect_guard(epoch):
                self._scene.remove_objects(
                    [PRODUCT_OBJECT_ID, CARRIER_OBJECT_ID, SAFETY_OBJECT_ID], cancel)
            with self._runner.side_effect_guard(epoch):
                self._tf.reset_to_identity()
        except ProcessCancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - ejection failure is a system failure
            self._fail_system(f'ejection failed: {exc}')
            return

        with self._runner.side_effect_guard(epoch):
            self._state.publish(WorkplaceStateDTO(in_place=False, product_data=None))
        self._logger.info('Ejection completed')
    # endregion ejection

    # region helpers
    def _fail_system(self, reason: str) -> None:
        self._logger.error(f'System failure: {reason}')
        self._on_system_failure(reason)

    @staticmethod
    def _check(cancel: threading.Event) -> None:
        if cancel.is_set():
            raise ProcessCancelledError('cancelled')

    @staticmethod
    def _require(client: Client | None) -> Client:
        if client is None:
            raise InjectionError('workplace process service is not active')
        return client
    # endregion helpers
