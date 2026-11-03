# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, is permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
"""Validated named arguments and ROS message conversion."""

from typing import Annotated, Any

from pydantic import (
    BaseModel,
    Field,
    model_validator,
    StrictBool,
    StrictFloat,
    StrictInt,
    StrictStr,
)

from repositories.enums.operation_repository import ArgumentTypeEnum
from repositories.msg import OperationRepositoryArg

_VALUE_FIELDS = {
    ArgumentTypeEnum.FLOAT: 'argument_value_float',
    ArgumentTypeEnum.INT: 'argument_value_int',
    ArgumentTypeEnum.STR: 'argument_value_str',
    ArgumentTypeEnum.BOOL: 'argument_value_bool',
    ArgumentTypeEnum.FLOAT_ARRAY: 'argument_value_float_array',
    ArgumentTypeEnum.INT_ARRAY: 'argument_value_int_array',
    ArgumentTypeEnum.STR_ARRAY: 'argument_value_str_array',
    ArgumentTypeEnum.BYTE_ARRAY: 'argument_value_byte_array',
}
_MESSAGE_DEFAULTS = {
    'argument_value_float': -1.0,
    'argument_value_int': -1,
    'argument_value_str': '',
    'argument_value_bool': False,
    'argument_value_float_array': [],
    'argument_value_int_array': [],
    'argument_value_str_array': [],
    'argument_value_byte_array': [],
}
_ARRAY_FIELDS = {
    'argument_value_float_array',
    'argument_value_int_array',
    'argument_value_str_array',
    'argument_value_byte_array',
}
ByteValue = Annotated[StrictInt, Field(ge=0, le=255)]


class ArgumentDTO(BaseModel, frozen=True):
    """Validated named argument with exactly one typed value."""

    argument_name: StrictStr = Field(description='Name of the argument.')
    argument_type: ArgumentTypeEnum = Field(description='Type of the active value field.')
    argument_value_float: StrictFloat | None = Field(
        default=None, description='Active floating-point value.'
    )
    argument_value_int: StrictInt | None = Field(default=None, description='Active integer value.')
    argument_value_str: StrictStr | None = Field(
        default=None, description='Active string value.'
    )
    argument_value_bool: StrictBool | None = Field(
        default=None, description='Active boolean value.'
    )
    argument_value_float_array: list[StrictFloat] | None = Field(
        default=None, description='Active floating-point array.'
    )
    argument_value_int_array: list[StrictInt] | None = Field(
        default=None, description='Active integer array.'
    )
    argument_value_str_array: list[StrictStr] | None = Field(
        default=None, description='Active string array.'
    )
    argument_value_byte_array: list[ByteValue] | None = Field(
        default=None, description='Active byte array.'
    )

    @model_validator(mode='after')
    def _validate_active_value(self) -> 'ArgumentDTO':
        active_field = _VALUE_FIELDS[self.argument_type]
        for field_name in _VALUE_FIELDS.values():
            value = getattr(self, field_name)
            if field_name == active_field and value is None:
                raise ValueError(f'{active_field} is required for {self.argument_type.value}')
            if field_name != active_field and value is not None:
                raise ValueError(f'{field_name} must be empty when {active_field} is active')
        return self

    @classmethod
    def from_named_value(cls, name: str, value: Any) -> 'ArgumentDTO':
        """Infer the argument type from a non-ambiguous JSON value."""
        if type(value) is bool:
            argument_type = ArgumentTypeEnum.BOOL
        elif type(value) is int:
            argument_type = ArgumentTypeEnum.INT
        elif type(value) is float:
            argument_type = ArgumentTypeEnum.FLOAT
        elif type(value) is str:
            argument_type = ArgumentTypeEnum.STR
        elif isinstance(value, list):
            if not value:
                raise ValueError(
                    f'Cannot infer the element type of empty array argument {name!r}'
                )
            if all(type(item) is bool for item in value):
                raise ValueError(f'Boolean arrays are not supported for argument {name!r}')
            if all(type(item) is int for item in value):
                argument_type = ArgumentTypeEnum.INT_ARRAY
            elif all(type(item) is float for item in value):
                argument_type = ArgumentTypeEnum.FLOAT_ARRAY
            elif all(type(item) is str for item in value):
                argument_type = ArgumentTypeEnum.STR_ARRAY
            else:
                raise ValueError(
                    f'Argument {name!r} must be a homogeneous supported array'
                )
        else:
            raise ValueError(
                f'Argument {name!r} has unsupported value type {type(value).__name__}'
            )
        return cls.model_validate(
            {
                'argument_name': name,
                'argument_type': argument_type,
                _VALUE_FIELDS[argument_type]: value,
            }
        )

    @classmethod
    def from_msg(cls, message: OperationRepositoryArg) -> 'ArgumentDTO':
        """Validate a ROS message, including inactive fields being defaulted."""
        try:
            argument_type = ArgumentTypeEnum(message.argument_type)
        except ValueError as error:
            raise ValueError(
                f'Unsupported argument_type {message.argument_type!r}'
            ) from error

        active_field = _VALUE_FIELDS[argument_type]
        for field_name, default in _MESSAGE_DEFAULTS.items():
            value = getattr(message, field_name)
            is_default = len(value) == 0 if field_name in _ARRAY_FIELDS else value == default
            if field_name != active_field and not is_default:
                raise ValueError(
                    f'{field_name} must be at its message default when '
                    f'{active_field} is active'
                )

        active_value = getattr(message, active_field)
        if active_field in _ARRAY_FIELDS:
            active_value = list(active_value)
        return cls.model_validate(
            {
                'argument_name': message.argument_name,
                'argument_type': argument_type,
                active_field: active_value,
            }
        )

    def to_msg(self) -> OperationRepositoryArg:
        """Convert the argument to a ROS message with inactive fields defaulted."""
        message = OperationRepositoryArg()
        message.argument_name = self.argument_name
        message.argument_type = self.argument_type.value
        active_field = _VALUE_FIELDS[self.argument_type]
        setattr(message, active_field, getattr(self, active_field))
        return message
