# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
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

"""Frame handling for the pattern poses of an optimization request."""

from collections.abc import Sequence

from execution_optimizer.exceptions import OptimizationError
from execution_optimizer.model.pattern_points_dto import PatternPointsDTO


def common_frame_id(patterns: Sequence[PatternPointsDTO]) -> str:
    """Return the single frame shared by all pattern poses.

    :raises OptimizationError: If any pose has an empty frame or more than one frame is used.
    """
    frames = {frame_id for pattern in patterns for frame_id in pattern.frame_ids}
    if '' in frames or not frames:
        empty = [pattern.id for pattern in patterns if not pattern.frame_ids or '' in pattern.frame_ids]
        raise OptimizationError(f'Pattern poses without a frame id: {empty}')
    if len(frames) > 1:
        raise OptimizationError(
            f'Pattern poses must share one frame, got {sorted(frames)}'
        )
    return next(iter(frames))
