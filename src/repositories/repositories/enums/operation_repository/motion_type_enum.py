# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# BSD-3-Clause license applies.
"""Supported dispensing movement types."""

from enum import Enum


class MotionTypeEnum(str, Enum):
    """Supported movement planner modes."""

    LIN = 'LIN'
    PTP = 'PTP'
    CIRC = 'CIRC'
