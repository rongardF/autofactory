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
"""A pose given as position and roll-pitch-yaw orientation."""

from geometry_msgs.msg import PoseStamped
from pydantic import BaseModel, ConfigDict, Field

from station_workplace.utils.transformations import pose6_to_pose


class PoseRpyDTO(BaseModel):
    """Position [m] and roll-pitch-yaw orientation [rad] of a frame."""

    model_config = ConfigDict(frozen=True)

    position: tuple[float, float, float] = Field(..., description='Position [x, y, z] [m].')
    orientation: tuple[float, float, float] = Field(
        ..., description='Orientation [roll, pitch, yaw] [rad], ZYX composition as in tf2.'
    )

    def to_pose_stamped(self, frame_id: str) -> PoseStamped:
        """Return the pose expressed in ``frame_id``."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose = pose6_to_pose([*self.position, *self.orientation])
        return pose
