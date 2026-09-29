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
"""TrajectoryGrouper — groups trajectory paths into blended execution groups."""

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class TrajectoryGrouper:
    """Stateless utility that implements the D-07 blend grouping algorithm."""

    @staticmethod
    def _effective_blend_radius(path: TrajectoryPathDTO) -> float:
        """Return the blend radius used for grouping decisions.

        ``PTP`` paths are planned by OMPL, which cannot blend, so their
        ``blend_radius`` is ignored (treated as ``0.0``) for grouping.  All
        other motion types use their declared ``blend_radius``.

        :param path: Trajectory path to evaluate.
        :type path: TrajectoryPathDTO
        :returns: ``0.0`` for ``PTP`` paths, otherwise ``path.blend_radius``.
        :rtype: float
        """
        if path.motion_type == MotionTypeEnum.PTP:
            return 0.0
        return path.blend_radius

    @staticmethod
    def group(paths: list[TrajectoryPathDTO]) -> list[list[TrajectoryPathDTO]]:
        """Group trajectory paths into blended execution groups.

        Consecutive paths where the *previous* path has an effective
        ``blend_radius > 0`` are merged into one group so they can be submitted
        as a single PILZ ``MotionSequenceRequest``.  A path whose predecessor
        has an effective ``blend_radius <= 0`` starts a new single-item group.

        ``PTP`` paths are always isolated into their own single-item group
        (D-2): they are planned by OMPL rather than PILZ, cannot blend, and
        must not share a group with ``LIN``/``CIRC`` paths.  A ``PTP`` therefore
        closes any open group before it and forces the following path to start a
        new group; its ``blend_radius`` is ignored.

        :param paths: Non-empty list of validated :class:`~movement_controller.models.TrajectoryPathDTO`
            objects with unique ``path_id`` values (guaranteed by
            :class:`~movement_controller.models.TrajectoryGoalDTO` validation).
        :type paths: list[TrajectoryPathDTO]
        :returns: Ordered list of groups, where each group is a list of
            :class:`~movement_controller.models.TrajectoryPathDTO` objects.
        :rtype: list[list[TrajectoryPathDTO]]
        :raises ValueError: If ``paths`` is empty.
        """
        if not paths:
            raise ValueError('paths list must not be empty')

        # Grouping loop
        groups: list[list[TrajectoryPathDTO]] = []
        for i, path in enumerate(paths):
            if i == 0:
                # First path always starts a new group.
                groups.append([path])
            elif path.motion_type == MotionTypeEnum.PTP:
                # PTP is always isolated into its own single-item group (D-2).
                groups.append([path])
            elif TrajectoryGrouper._effective_blend_radius(groups[-1][-1]) > 0:
                # Predecessor requested a blend (and is not a PTP): merge.
                groups[-1].append(path)
            else:
                # Predecessor closed its group (or was an isolated PTP): new group.
                groups.append([path])

        return groups
