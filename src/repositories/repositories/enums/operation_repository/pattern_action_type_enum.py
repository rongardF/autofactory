# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# BSD-3-Clause license applies.
"""Supported dispensing pattern actions."""

from enum import Enum


class PatternActionTypeEnum(str, Enum):
    """Action taken at a movement boundary."""

    OPEN_VALVE = 'open_valve'
    CLOSE_VALVE = 'close_valve'
    WAIT = 'wait'
