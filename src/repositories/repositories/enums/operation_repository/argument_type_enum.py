# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Supported operation argument value types."""

from enum import Enum


class ArgumentTypeEnum(str, Enum):
    """Supported value types for named operation arguments."""

    FLOAT = 'float'
    INT = 'int'
    STR = 'str'
    BOOL = 'bool'
    FLOAT_ARRAY = 'float_array'
    INT_ARRAY = 'int_array'
    STR_ARRAY = 'str_array'
    BYTE_ARRAY = 'byte_array'
