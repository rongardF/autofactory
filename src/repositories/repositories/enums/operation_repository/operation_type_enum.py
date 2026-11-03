# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# BSD-3-Clause license applies.
"""Supported operation types."""

from enum import Enum


class OperationTypeEnum(str, Enum):
    """Supported values for the operation type."""

    DISPENSING = 'dispensing'
