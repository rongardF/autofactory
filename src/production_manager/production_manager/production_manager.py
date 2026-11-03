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

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.service import Service

from production_manager.model.production_manager_node_config import ProductionManagerNodeConfig
from production_manager.model.result_dto import ResultDTO
from production_manager.services.backend_manager import BackendManager
from production_manager.services.local_manager import LocalManager
from production_manager.utils.uuid_validator import is_uuid4
from production_manager.srv import GetProductionData, ReportProduction, SetOperation


class ProductionManager(LifecycleNode):
    def __init__(self, node_name: str = 'production_manager') -> None:
        super().__init__(node_name)

        self._config: ProductionManagerNodeConfig | None = None
        self._backend: LocalManager | BackendManager | None = None
        self._get_production_data_service: Service | None = None
        self._report_production_service: Service | None = None
        self._set_operation_service: Service | None = None
        self._service_callback_group = ReentrantCallbackGroup()

        self.declare_parameter(
            'local_manager',
            True,
            ParameterDescriptor(description='Use the local assembly file backend.'),
        )
        self.declare_parameter(
            'assembly_file_path',
            '/tmp/station/mock_assembly_file.json',
            ParameterDescriptor(description='Absolute path to the local assembly JSON file.'),
        )
        self.declare_parameter(
            'station_id',
            '',
            ParameterDescriptor(description='UUID4 identifying this production station.'),
        )
        self.declare_parameter(
            'manager_service_url',
            '',
            ParameterDescriptor(description='Base URL of the production manager REST service.'),
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring production manager...')
        try:
            config = ProductionManagerNodeConfig.model_validate(
                {
                    name: self.get_parameter(name).value
                    for name in (
                        'local_manager',
                        'assembly_file_path',
                        'station_id',
                        'manager_service_url',
                    )
                }
            )
            backend: LocalManager | BackendManager
            if config.local_manager:
                backend = LocalManager(config.assembly_file_path, self.get_logger())
            else:
                backend = BackendManager(
                    config.manager_service_url,
                    config.station_id,
                    self.get_logger(),
                )
            self._config = config
            self._backend = backend
            self.get_logger().info('Production manager configured.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            self.get_logger().error(f'Unable to configure production manager: {error}')
            self._config = None
            self._backend = None
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating production manager...')
        if self._config is None or self._backend is None:
            self.get_logger().error('Cannot activate before successful configuration.')
            return TransitionCallbackReturn.FAILURE
        try:
            self._get_production_data_service = self.create_service(
                GetProductionData,
                '~/get_production_data',
                self._get_production_data_cb,
                callback_group=self._service_callback_group,
            )
            self._report_production_service = self.create_service(
                ReportProduction,
                '~/report_production',
                self._report_production_cb,
                callback_group=self._service_callback_group,
            )
            if self._config.local_manager:
                self._set_operation_service = self.create_service(
                    SetOperation,
                    '~/set_operation',
                    self._set_operation_cb,
                    callback_group=self._service_callback_group,
                )
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                self._destroy_services()
            return result
        except Exception as error:
            self.get_logger().error(f'Unable to activate production manager: {error}')
            self._destroy_services()
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating production manager...')
        try:
            services_removed = self._destroy_services()
            result = super().on_deactivate(state)
            if not services_removed:
                return TransitionCallbackReturn.FAILURE
            return result
        except Exception as error:
            self.get_logger().error(f'Unable to deactivate production manager: {error}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up production manager...')
        if not self._destroy_services():
            return TransitionCallbackReturn.FAILURE
        self._backend = None
        self._config = None
        return super().on_cleanup(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down production manager...')
        services_removed = self._destroy_services()
        self._backend = None
        self._config = None
        result = super().on_shutdown(state)
        if not services_removed:
            return TransitionCallbackReturn.FAILURE
        return result

    def _destroy_services(self) -> bool:
        all_removed = True
        for attribute in (
            '_get_production_data_service',
            '_report_production_service',
            '_set_operation_service',
        ):
            service = getattr(self, attribute)
            if service is not None:
                try:
                    destroyed = self.destroy_service(service)
                except Exception as error:
                    self.get_logger().error(f'Unable to destroy {attribute}: {error}')
                    all_removed = False
                    continue
                if destroyed:
                    setattr(self, attribute, None)
                else:
                    self.get_logger().error(f'Unable to destroy {attribute}.')
                    all_removed = False
        return all_removed

    def _get_production_data_cb(
        self,
        request: GetProductionData.Request,
        response: GetProductionData.Response,
    ) -> GetProductionData.Response:
        try:
            self.get_logger().info(
                f'Getting production data for product={request.product_number} '
                f'revision={request.product_revision} serial={request.serial_number}.'
            )
            if self._config is None or self._backend is None:
                raise RuntimeError('Production manager is not configured.')
            result = self._backend.get_production_data(
                request.product_number,
                request.product_revision,
                request.serial_number,
                self._config.station_id,
            )
            response.success = result.success
            response.error_message = result.error_message
            if result.success:
                response.assembly_id = result.assembly_id
                response.operation_id = result.operation_id
                response.assembly_model_id = result.assembly_model_id
            else:
                response.assembly_id = ''
                response.operation_id = ''
                response.assembly_model_id = ''
                self.get_logger().warning(f'Production data unavailable: {result.error_message}')
        except Exception as error:
            self.get_logger().error(f'get_production_data failed: {error}')
            response.assembly_id = ''
            response.operation_id = ''
            response.assembly_model_id = ''
            response.success = False
            response.error_message = str(error)
        return response

    def _report_production_cb(
        self,
        request: ReportProduction.Request,
        response: ReportProduction.Response,
    ) -> ReportProduction.Response:
        try:
            self.get_logger().info(
                f'Reporting production for assembly={request.assembly_id} '
                f'operation={request.operation_id} outcome={request.success}.'
            )
            if not is_uuid4(request.assembly_id):
                raise ValueError('assembly_id must be a valid UUID4.')
            if not is_uuid4(request.operation_id):
                raise ValueError('operation_id must be a valid UUID4.')
            if self._config is None or self._backend is None:
                raise RuntimeError('Production manager is not configured.')
            result = self._backend.report_production(
                request.assembly_id,
                request.operation_id,
                request.success,
                self._config.station_id,
            )
            self._set_result(response, result)
            if not result.success:
                self.get_logger().warning(f'Production report rejected: {result.error_message}')
        except Exception as error:
            self.get_logger().error(f'report_production failed: {error}')
            response.success = False
            response.error_message = str(error)
        return response

    def _set_operation_cb(
        self,
        request: SetOperation.Request,
        response: SetOperation.Response,
    ) -> SetOperation.Response:
        try:
            self.get_logger().info(
                f'Setting operation={request.operation_id} for serial={request.serial_number} '
                f'at station={request.station_id}.'
            )
            if not is_uuid4(request.operation_id):
                raise ValueError('operation_id must be a valid UUID4.')
            if not is_uuid4(request.station_id):
                raise ValueError('station_id must be a valid UUID4.')
            if not isinstance(self._backend, LocalManager):
                raise RuntimeError('set_operation is available only in local manager mode.')
            result = self._backend.set_operation(
                request.operation_id,
                request.serial_number,
                request.station_id,
            )
            self._set_result(response, result)
            if not result.success:
                self.get_logger().warning(f'Unable to set operation: {result.error_message}')
        except Exception as error:
            self.get_logger().error(f'set_operation failed: {error}')
            response.success = False
            response.error_message = str(error)
        return response

    def _set_result(
        self,
        response: ReportProduction.Response | SetOperation.Response,
        result: ResultDTO,
    ) -> None:
        response.success = result.success
        response.error_message = result.error_message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProductionManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
