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

from threading import Lock
from typing import Protocol

from production_manager.interface.production_backend import ProductionBackend
from production_manager.model.current_operation_dto import CurrentOperationDTO
from production_manager.model.production_data_dto import ProductionDataDTO
from production_manager.model.report_result_dto import ReportResultDTO
from production_manager.model.result_dto import ResultDTO
from production_manager.utils.assembly_reader import (
    find_operation,
    get_assembly_id,
    get_operation_start_model_id,
    get_product_number,
    get_product_revision,
    load_assembly_file,
    validate_assembly,
)
from production_manager.utils.uuid_validator import is_uuid4
from repositories.enums.operation_repository import SCHEMA_VERSION


class LoggerLike(Protocol):
    def info(self, message: str) -> object: ...


class LocalManager(ProductionBackend):
    def __init__(self, assembly_file_path: str, logger: LoggerLike) -> None:
        self._assembly = load_assembly_file(assembly_file_path)
        validate_assembly(self._assembly)
        self._logger = logger
        self._queue: dict[int, CurrentOperationDTO] = {}
        self._lock = Lock()

    def set_operation(
        self,
        index: int,
        operation_id: str,
        serial_number: str,
        station_id: str,
    ) -> ResultDTO:
        if index < 0:
            return ResultDTO.failure('index must be >= 0.')
        if not is_uuid4(operation_id):
            return ResultDTO.failure('operation_id must be a valid UUID4.')
        if not is_uuid4(station_id):
            return ResultDTO.failure('station_id must be a valid UUID4.')
        if find_operation(self._assembly, operation_id) is None:
            return ResultDTO.failure(f'Operation {operation_id!r} was not found in the assembly.')

        entry = CurrentOperationDTO(
            index=index,
            operation_id=operation_id,
            serial_number=serial_number,
            station_id=station_id,
        )
        with self._lock:
            queued_serial = self._queued_serial()
            if queued_serial is not None and queued_serial != serial_number:
                if index != 0:
                    return ResultDTO.failure(
                        'index must be 0 when starting the operation sequence of a new serial_number.'
                    )
                self._queue.clear()
            if index > len(self._queue):
                return ResultDTO.failure(
                    f'index {index} leaves a gap; the next free index is {len(self._queue)}.'
                )
            self._queue[index] = entry
        return ResultDTO.ok()

    def get_production_data(
        self,
        product_number: str,
        product_revision: str,
        serial_number: str,
        station_id: str,
        schema_version: str = SCHEMA_VERSION,
    ) -> ProductionDataDTO:
        """Return local production data; schema version is checked on assembly use."""
        del schema_version
        with self._lock:
            queue = dict(self._queue)
        if not queue:
            return ProductionDataDTO.failure('No operation has been set.')
        if serial_number != self._queued_serial(queue):
            return ProductionDataDTO.failure('serial_number does not match the stored operation.')
        if any(entry.station_id != station_id for entry in queue.values()):
            return ProductionDataDTO.failure(
                'The stored operation is assigned to a different station_id.'
            )
        if product_number != get_product_number(self._assembly):
            return ProductionDataDTO.failure('product_number does not match the assembly file.')
        if product_revision != get_product_revision(self._assembly):
            return ProductionDataDTO.failure('product_revision does not match the assembly file.')

        pending = [queue[index] for index in sorted(queue) if not queue[index].done]
        if not pending:
            return ProductionDataDTO.failure('No pending operation.')
        current = pending[0]
        operation = find_operation(self._assembly, current.operation_id)
        if operation is None:
            return ProductionDataDTO.failure(
                f'Operation {current.operation_id!r} was not found in the assembly.'
            )
        return ProductionDataDTO(
            assembly_id=get_assembly_id(self._assembly),
            operation_id=current.operation_id,
            assembly_model_id=get_operation_start_model_id(self._assembly, operation),
            success=True,
        )

    def report_production(
        self,
        assembly_id: str,
        operation_id: str,
        success: bool,
        station_id: str,
        reason: str = '',
    ) -> ReportResultDTO:
        if not is_uuid4(assembly_id):
            return ReportResultDTO.failure('assembly_id must be a valid UUID4.')
        if not is_uuid4(operation_id):
            return ReportResultDTO.failure('operation_id must be a valid UUID4.')
        if assembly_id != get_assembly_id(self._assembly):
            return ReportResultDTO.failure('assembly_id does not match the assembly file.')
        with self._lock:
            if not self._queue:
                return ReportResultDTO.failure('No operation has been set.')
            reported = next(
                (
                    self._queue[index]
                    for index in sorted(self._queue)
                    if self._queue[index].operation_id == operation_id
                    and not self._queue[index].done
                ),
                None,
            )
            if reported is None:
                return ReportResultDTO.failure(
                    'operation_id does not match any queued operation.'
                )
            self._queue[reported.index] = reported.model_copy(update={'done': True})
            finished_in_station = (not success) or not any(
                index > reported.index for index in self._queue
            )
            if finished_in_station:
                self._queue.clear()

        if success:
            self._logger.info(
                f'Reported operation {operation_id} for assembly {assembly_id} with '
                f'outcome success=True at station {station_id}.'
            )
        else:
            self._logger.info(
                f'Reported operation {operation_id} for assembly {assembly_id} with '
                f'outcome success=False at station {station_id}; reason: {reason}.'
            )
        return ReportResultDTO.ok(finished_in_station)

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()

    def _queued_serial(self, queue: dict[int, CurrentOperationDTO] | None = None) -> str | None:
        entries = self._queue if queue is None else queue
        return next(iter(entries.values())).serial_number if entries else None
