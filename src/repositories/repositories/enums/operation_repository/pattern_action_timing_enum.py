# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# BSD-3-Clause license applies.
"""Supported timing values for pattern actions."""

from enum import Enum


class PatternActionTimingEnum(str, Enum):
    """Boundary of a movement at which an action is executed."""

    START = 'start'
    END = 'end'
