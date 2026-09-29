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
"""TrajectoryGoalDTO — Pydantic DTO wrapping the ExecuteTrajectory goal."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, field_validator

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO

from movement_controller.action import ExecuteTrajectory


class TrajectoryGoalDTO(BaseModel):
    """Validated, immutable goal containing an ordered list of trajectory paths."""

    model_config = ConfigDict(frozen=True)

    paths: list[TrajectoryPathDTO] = Field(
        description='Non-empty ordered list of trajectory paths to execute'
    )

    @field_validator('paths', mode='after')
    @classmethod
    def validate_paths(cls, v: list[TrajectoryPathDTO]) -> list[TrajectoryPathDTO]:
        """Validate that ``paths`` is non-empty and contains no duplicate IDs.

        :param v: The parsed list of :class:`TrajectoryPathDTO` objects.
        :type v: list[TrajectoryPathDTO]
        :returns: The validated list, unchanged.
        :rtype: list[TrajectoryPathDTO]
        :raises ValueError: If ``paths`` is empty or contains a duplicate ``path_id``.
        """
        if not v:
            raise ValueError('paths must not be empty')
        seen: set[str] = set()
        for path in v:
            if path.path_id in seen:
                raise ValueError(f'duplicate path_id: {path.path_id!r}')
            seen.add(path.path_id)
        return v

    @classmethod
    def from_ros_msg(cls, goal_msg: ExecuteTrajectory.Goal) -> TrajectoryGoalDTO:
        """Construct a :class:`TrajectoryGoalDTO` from an :class:`ExecuteTrajectory.Goal` ROS 2 message.

        :param goal_msg: ROS 2 action goal message containing a list of
            :class:`~movement_controller.msg.TrajectoryPath` messages.
        :type goal_msg: ExecuteTrajectory.Goal
        :returns: Validated, immutable :class:`TrajectoryGoalDTO`.
        :rtype: TrajectoryGoalDTO
        :raises pydantic.ValidationError: If any path fails Pydantic field validation.
        :raises ValueError: If the resulting paths list is empty or contains
            duplicate ``path_id`` values.
        """
        return cls(paths=[TrajectoryPathDTO.from_ros_msg(p) for p in goal_msg.paths])
