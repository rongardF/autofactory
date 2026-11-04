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
"""Enums and the schema version for operation data."""

from repositories.enums.operation_repository.argument_type_enum import ArgumentTypeEnum
from repositories.enums.operation_repository.circ_type_enum import CircTypeEnum
from repositories.enums.operation_repository.motion_type_enum import MotionTypeEnum
from repositories.enums.operation_repository.operation_type_enum import OperationTypeEnum
from repositories.enums.operation_repository.pattern_action_timing_enum import (
    PatternActionTimingEnum,
)
from repositories.enums.operation_repository.pattern_action_type_enum import (
    PatternActionTypeEnum,
)
from repositories.enums.operation_repository.schema_version import SCHEMA_VERSION

__all__ = [
    'SCHEMA_VERSION',
    'ArgumentTypeEnum',
    'CircTypeEnum',
    'MotionTypeEnum',
    'OperationTypeEnum',
    'PatternActionTimingEnum',
    'PatternActionTypeEnum',
]
