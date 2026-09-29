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


from moveit_msgs.msg import MotionSequenceResponse


class PlanResultDTO(BaseModel):
    """Internal immutable result returned by PilzPlannerService.plan().

    Uses ``arbitrary_types_allowed=True`` so that a ``RobotTrajectory`` instance
    (a non-Pydantic C++ extension type) can be stored in the ``trajectory`` field.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    success: bool = Field(description='True if PILZ planning succeeded', default=False)
    error_message: str = Field(
        default='',
        description='Human-readable error; empty on success',
    )
    motion_plan: MotionSequenceResponse | None = Field(
        default=None,
        description='Full MotionSequenceResponse from PILZ; empty on failure',
    )
    path_ids: list[str] = Field(
        default_factory=list,
        description='path IDs this result covers, in execution order; populated by plan_all(), empty in plan() results',
    )
    blended: bool = Field(
        default=False,
        description='True if this group has more than one path (blend group); False for size-1 groups',
    )
