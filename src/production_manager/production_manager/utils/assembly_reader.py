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
"""Read selected assembly fields without binding the file to a DTO.

The root schema version governs root fields and operation-level fields.
Operation schemas and patterns are intentionally not interpreted here.
"""

import json
from pathlib import Path
from typing import Any

from production_manager.enumerator.schema_version_enum import SchemaVersionEnum
from production_manager.exception import (
    AssemblyFileException,
    UnsupportedSchemaVersionException,
)
from production_manager.utils.uuid_validator import is_uuid4


def load_assembly_file(path: str) -> dict[str, Any]:
    try:
        with Path(path).open(encoding='utf-8') as file:
            assembly = json.load(file)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AssemblyFileException(f'Unable to read assembly JSON file {path!r}: {error}') from error
    if not isinstance(assembly, dict):
        raise AssemblyFileException('Assembly JSON root must be an object.')
    return assembly


def get_schema_version(assembly: dict[str, Any]) -> SchemaVersionEnum:
    if 'schema_version' not in assembly:
        raise AssemblyFileException('Assembly is missing root key "schema_version".')
    try:
        return SchemaVersionEnum(assembly['schema_version'])
    except (TypeError, ValueError) as error:
        raise UnsupportedSchemaVersionException(
            f'Unsupported assembly schema_version: {assembly["schema_version"]!r}.'
        ) from error


def _required_string(mapping: dict[str, Any], key: str, context: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise AssemblyFileException(f'{context} key {key!r} must be a non-empty string.')
    return value


def get_assembly_id(assembly: dict[str, Any]) -> str:
    match get_schema_version(assembly):
        case SchemaVersionEnum.V1_0:
            value = _required_string(assembly, 'id', 'Assembly')
            if not is_uuid4(value):
                raise AssemblyFileException('Assembly key "id" must be a valid UUID4.')
            return value


def get_product_number(assembly: dict[str, Any]) -> str:
    match get_schema_version(assembly):
        case SchemaVersionEnum.V1_0:
            return _required_string(assembly, 'product_number', 'Assembly')


def get_product_revision(assembly: dict[str, Any]) -> str:
    match get_schema_version(assembly):
        case SchemaVersionEnum.V1_0:
            return _required_string(assembly, 'product_revision', 'Assembly')


def get_operations(assembly: dict[str, Any]) -> list[dict[str, Any]]:
    match get_schema_version(assembly):
        case SchemaVersionEnum.V1_0:
            operations = assembly.get('operations')
            if not isinstance(operations, list) or not all(
                isinstance(operation, dict) for operation in operations
            ):
                raise AssemblyFileException('Assembly key "operations" must be a list of objects.')
            return operations


def find_operation(assembly: dict[str, Any], operation_id: str) -> dict[str, Any] | None:
    match get_schema_version(assembly):
        case SchemaVersionEnum.V1_0:
            for operation in get_operations(assembly):
                if operation.get('id') == operation_id:
                    return operation
            return None


def get_operation_start_model_id(
    assembly: dict[str, Any],
    operation: dict[str, Any],
) -> str:
    match get_schema_version(assembly):
        case SchemaVersionEnum.V1_0:
            value = _required_string(operation, 'start_assembly_model_id', 'Operation')
            if not is_uuid4(value):
                raise AssemblyFileException(
                    'Operation key "start_assembly_model_id" must be a valid UUID4.'
                )
            return value


def validate_assembly(assembly: dict[str, Any]) -> None:
    get_schema_version(assembly)
    get_assembly_id(assembly)
    get_product_number(assembly)
    get_product_revision(assembly)
    for operation in get_operations(assembly):
        operation_id = _required_string(operation, 'id', 'Operation')
        if not is_uuid4(operation_id):
            raise AssemblyFileException('Operation key "id" must be a valid UUID4.')
        get_operation_start_model_id(assembly, operation)
