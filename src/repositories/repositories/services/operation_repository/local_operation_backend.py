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
"""Local operation source backed by an assembly JSON file."""

import json
from pathlib import Path
from typing import Any

from repositories.enums.operation_repository import SCHEMA_VERSION
from repositories.interfaces.operation_repository.operation_backend import (
    OperationBackend,
)
from repositories.models.operation_repository.operation_dto import OperationDTO
from repositories.utils.uuid_validator import parse_uuid4


class LocalOperationBackend(OperationBackend):
    """Load, validate and cache all operations during configuration."""

    def __init__(self, assembly_file_path: str) -> None:
        self._assembly_file_path = Path(assembly_file_path)
        try:
            with self._assembly_file_path.open(encoding='utf-8') as file:
                assembly: Any = json.load(file)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise ValueError(
                f'Unable to read assembly JSON file '
                f'{str(self._assembly_file_path)!r}: {error}'
            ) from error
        if not isinstance(assembly, dict):
            raise ValueError('Assembly JSON root must be an object.')

        schema_version = assembly.get('schema_version')
        if schema_version != SCHEMA_VERSION:
            raise ValueError(
                f'Unsupported assembly schema_version: {schema_version!r}; '
                f'expected {SCHEMA_VERSION!r}.'
            )
        try:
            self._assembly_id = parse_uuid4(assembly['id'], 'assembly id')
        except (KeyError, ValueError, TypeError) as error:
            raise ValueError(f'Assembly root id is invalid: {error}') from error

        operations = assembly.get('operations')
        if not isinstance(operations, list) or not all(
            isinstance(operation, dict) for operation in operations
        ):
            raise ValueError('Assembly key "operations" must be a list of objects.')

        parsed_operations: dict[str, OperationDTO] = {}
        for index, operation in enumerate(operations):
            try:
                dto = OperationDTO.from_operation_data(operation)
                operation_id = str(dto.id)
                if operation_id in parsed_operations:
                    raise ValueError(f'duplicate operation id {dto.id!r}')
                parsed_operations[operation_id] = dto
            except Exception as error:
                raise ValueError(
                    f'Invalid operation at index {index}: {error}'
                ) from error
        self._operations = parsed_operations

    @property
    def operation_count(self) -> int:
        """Return the number of validated operations in the assembly."""
        return len(self._operations)

    def get_operation(self, operation_id: str, assembly_id: str) -> OperationDTO:
        """Look up one cached operation after checking its assembly identifier."""
        if assembly_id != self._assembly_id:
            raise LookupError(
                f'assembly_id {assembly_id!r} does not match the assembly file'
            )
        try:
            return self._operations[operation_id]
        except KeyError as error:
            raise LookupError(
                f'operation {operation_id!r} was not found in assembly '
                f'{self._assembly_id!r}'
            ) from error
