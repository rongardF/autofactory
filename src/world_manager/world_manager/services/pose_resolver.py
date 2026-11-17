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
"""Resolves poses given in arbitrary TF frames into the Gazebo world frame."""

from typing import Callable

from world_manager.exceptions import ModelValidationError
from world_manager.models import PoseDTO
from world_manager.utils import compose

# (target_frame, source_frame, timeout_sec) -> pose of source_frame in target_frame; raises on failure
TfLookup = Callable[[str, str, float], PoseDTO]


class PoseResolver:
    """Turns a pose in any TF frame into a pose in the world frame."""

    def __init__(self, lookup: TfLookup, world_frame: str, timeout_sec: float) -> None:
        """``lookup`` is the TF seam: a tf2 buffer in the node, a fake in unit tests."""
        self._lookup = lookup
        self._world_frame = world_frame
        self._timeout_sec = timeout_sec

    def to_world(self, pose: PoseDTO) -> PoseDTO:
        """Return ``pose`` expressed in the world frame; raises ModelValidationError if TF cannot resolve it."""
        if pose.frame_id in ('', self._world_frame):
            return pose.model_copy(update={'frame_id': self._world_frame})
        try:
            frame_in_world = self._lookup(self._world_frame, pose.frame_id, self._timeout_sec)
            return compose(frame_in_world, pose, self._world_frame)
        except Exception as e:
            raise ModelValidationError(
                f"cannot resolve frame '{pose.frame_id}' to '{self._world_frame}': {e}") from e
