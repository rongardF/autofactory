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

"""Package-internal exceptions.

They never cross a node boundary: the service callback converts them into a
failure result.
"""

from enum import Enum


class GeometryFailureEnum(str, Enum):
    """Reasons a single pattern cannot be turned into a region of interest."""

    NON_PLANAR = 'non_planar'
    HORIZONTAL_NORMAL = 'horizontal_normal'


class OptimizationError(Exception):
    """The optimization request failed; the message is the user-facing error."""


class PatternGeometryError(Exception):
    """A pattern has an unusable geometry."""

    def __init__(self, pattern_id: str, kind: GeometryFailureEnum) -> None:
        """Remember which pattern failed and why.

        :param pattern_id: Id of the offending pattern.
        :type pattern_id: str
        :param kind: Failure class.
        :type kind: GeometryFailureEnum
        """
        super().__init__(f'pattern {pattern_id!r}: {kind.value}')
        self.pattern_id = pattern_id
        self.kind = kind


class CameraInfoError(Exception):
    """The camera intrinsics could not be obtained or are unusable."""
