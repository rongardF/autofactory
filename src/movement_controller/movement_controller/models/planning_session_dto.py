# Copyright (c) 2026, Movement Controller Contributors
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
"""PlanResultDTO — internal result type for PilzPlannerService.plan()."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from moveit_msgs.msg import RobotState

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class PlanningSessionDTO(BaseModel):
    """Internal mutable data object to keep track of dynamic data/state during a planning session.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=False)

    groups: list[list[TrajectoryPathDTO]] = Field(
        description='Groups of trajectory paths for the planning session',
        default_factory=list,
    )
    current_group: list[TrajectoryPathDTO] = Field(
        description='The current group of trajectory paths being planned/executed',
        default_factory=list,
    )
    last_predicted_state: RobotState | None = Field(
        description='The last predicted RobotState after executing a planned trajectory; used as the start state for the next plan',
        default=None,
    )

    def get_next_group(self) -> list[TrajectoryPathDTO] | None:
        """Pop and return the next group of paths to plan, updating :attr:`current_group`.

        Mutates the session by removing the first group from :attr:`groups` and
        storing it in :attr:`current_group`.

        :returns: The next group of :class:`TrajectoryPathDTO` objects, or
            ``None`` when all groups have been consumed.
        :rtype: list[TrajectoryPathDTO] | None
        """
        if self.groups:
            self.current_group = self.groups.pop(0)
            return self.current_group
        return None
