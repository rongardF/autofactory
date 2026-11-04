# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# BSD-3-Clause license applies.
"""Supported circular movement point types."""

from enum import Enum


class CircTypeEnum(str, Enum):
    """Interpretation of the circular movement point."""

    INTERIM = 'interim'
    CENTER = 'center'
