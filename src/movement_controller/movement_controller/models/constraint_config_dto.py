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
"""ConstraintConfigDTO — Pydantic v2 DTO for persistent motion constraints."""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field, model_validator

from movement_controller.enums import MotionTypeEnum
from movement_controller.models import TrajectoryGoalDTO


class ConstraintConfigDTO(BaseModel):
    """Validated, immutable container for all node-level motion constraint parameters."""

    model_config = ConfigDict(frozen=True)

    # Workspace bounding box (float; sentinels -1e9 / +1e9 = unconstrained per D-12)
    x_min: float = Field(default=-1e9, description='Workspace bounding box x lower bound (m). Sentinel -1e9 = unconstrained.')
    x_max: float = Field(default=1e9, description='Workspace bounding box x upper bound (m). Sentinel +1e9 = unconstrained.')
    y_min: float = Field(default=-1e9, description='Workspace bounding box y lower bound (m). Sentinel -1e9 = unconstrained.')
    y_max: float = Field(default=1e9, description='Workspace bounding box y upper bound (m). Sentinel +1e9 = unconstrained.')
    z_min: float = Field(default=-1e9, description='Workspace bounding box z lower bound (m). Sentinel -1e9 = unconstrained.')
    z_max: float = Field(default=1e9, description='Workspace bounding box z upper bound (m). Sentinel +1e9 = unconstrained.')

    # Orientation constraint (default 2*pi = unconstrained per D-04)
    orientation_tolerance_x: float = Field(
        default=2 * math.pi,
        description='Orientation tolerance around x axis (radians). Default 2π = unconstrained.',
    )
    orientation_tolerance_y: float = Field(
        default=2 * math.pi,
        description='Orientation tolerance around y axis (radians). Default 2π = unconstrained.',
    )
    orientation_tolerance_z: float = Field(
        default=2 * math.pi,
        description='Orientation tolerance around z axis (radians). Default 2π = unconstrained.',
    )

    # Speed/acceleration caps (0.0 = unconstrained per D-04)
    max_cartesian_speed: float = Field(
        default=0.0,
        description='Node-level max cartesian speed cap. 0.0 = unconstrained. Goals with path.cartesian_speed exceeding this are rejected at _goal_callback.',
    )
    max_cartesian_acceleration: float = Field(
        default=0.0,
        description='Node-level max cartesian acceleration cap. 0.0 = unconstrained. Goals with path.cartesian_acceleration exceeding this are rejected at _goal_callback.',
    )
    max_joint_speed: float = Field(
        default=0.0,
        description='Node-level max joint speed cap. 0.0 = unconstrained. Goals with path.joint_speed exceeding this are rejected at _goal_callback.',
    )
    max_joint_acceleration: float = Field(
        default=0.0,
        description='Node-level max joint acceleration cap. 0.0 = unconstrained. Goals with path.joint_acceleration exceeding this are rejected at _goal_callback.',
    )

    @model_validator(mode='after')
    def _validate_workspace_bounds(self) -> 'ConstraintConfigDTO':
        """Ensure workspace bounds are ordered correctly on each axis."""
        if self.x_min > self.x_max:
            raise ValueError(
                f'x_min ({self.x_min}) must be <= x_max ({self.x_max})'
            )
        if self.y_min > self.y_max:
            raise ValueError(
                f'y_min ({self.y_min}) must be <= y_max ({self.y_max})'
            )
        if self.z_min > self.z_max:
            raise ValueError(
                f'z_min ({self.z_min}) must be <= z_max ({self.z_max})'
            )
        return self

    @property
    def workspace_enabled(self) -> bool:
        """Check whether the workspace bounding-box constraint is active.

        The constraint is considered active when the configured range on at
        least one axis is tighter than the full sentinel range of ``2e9`` m
        (i.e. bounds are not both at the ``±1e9`` sentinel values).

        :returns: ``True`` when the workspace bound is tighter than the full
            sentinel range on at least one axis.
        :rtype: bool
        """
        return not (
            self.x_max - self.x_min >= 2e9
            and self.y_max - self.y_min >= 2e9
            and self.z_max - self.z_min >= 2e9
        )

    @property
    def orientation_constraint_enabled(self) -> bool:
        """Check whether the orientation constraint is active.

        :returns: ``True`` when any tolerance axis is tighter than ``2π`` radians
            (the default unconstrained value).
        :rtype: bool
        """
        return (
            self.orientation_tolerance_x < 2 * math.pi
            or self.orientation_tolerance_y < 2 * math.pi
            or self.orientation_tolerance_z < 2 * math.pi
        )
    
    def validate_goal(self, trajectory_goal: TrajectoryGoalDTO) -> None:
        """Validate all paths in a goal against the node-level speed/acceleration caps.

        Checks each path's ``cartesian_speed``, ``cartesian_acceleration``,
        ``joint_speed``, and ``joint_acceleration`` against the configured maxima.
        A limit is only enforced when both the path value and the configured cap
        are non-zero (``0.0`` is treated as unconstrained on either side).

        :param trajectory_goal: Goal whose paths are to be validated.
        :type trajectory_goal: TrajectoryGoalDTO
        :raises ValueError: If any path exceeds a configured speed or
            acceleration cap, with a human-readable message identifying the
            offending path ID, value, and cap.
        """
        for path in trajectory_goal.paths:
            if (
                path.motion_type in [MotionTypeEnum.LIN, MotionTypeEnum.CIRC]
                and path.cartesian_speed > 0.0
                and self.max_cartesian_speed > 0.0
                and path.cartesian_speed > self.max_cartesian_speed
            ):
                raise ValueError(
                    f"Path '{path.path_id}' cartesian_speed {path.cartesian_speed} m/s "
                    f"exceeds node maximum {self.max_cartesian_speed} m/s "
                    f"(constraints.max_cartesian_speed)"
                )

            if (
                path.motion_type in [MotionTypeEnum.LIN, MotionTypeEnum.CIRC]
                and path.cartesian_acceleration > 0.0
                and self.max_cartesian_acceleration > 0.0
                and path.cartesian_acceleration > self.max_cartesian_acceleration
            ):
                raise ValueError(
                    f"Path '{path.path_id}' cartesian_acceleration {path.cartesian_acceleration} m/s² "
                    f"exceeds node maximum {self.max_cartesian_acceleration} m/s² "
                    f"(constraints.max_cartesian_acceleration)"
                )
            
            if (
                path.motion_type in [MotionTypeEnum.PTP]
                and path.joint_speed > 0.0
                and self.max_joint_speed > 0.0
                and path.joint_speed > self.max_joint_speed
            ):
                raise ValueError(
                    f"Path '{path.path_id}' joint_speed {path.joint_speed} rad/s "
                    f"exceeds node maximum {self.max_joint_speed} rad/s "
                    f"(constraints.max_joint_speed)"
                )

            if (
                path.motion_type in [MotionTypeEnum.PTP]
                and path.joint_acceleration > 0.0
                and self.max_joint_acceleration > 0.0
                and path.joint_acceleration > self.max_joint_acceleration
            ):
                raise ValueError(
                    f"Path '{path.path_id}' joint_acceleration {path.joint_acceleration} rad/s² "
                    f"exceeds node maximum {self.max_joint_acceleration} rad/s² "
                    f"(constraints.max_joint_acceleration)"
                )
