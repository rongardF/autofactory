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

"""Grouping of dispensing patterns into groups that share one capture pose.

Pure Python / numpy: no ROS imports, so the whole procedure is testable on its own.

Minimising the number of groups is NP-hard, so a deterministic heuristic is used:
first-fit over a fixed set of pattern orderings, each followed by an elimination
pass that tries to empty the smallest groups; the result with the fewest groups
wins (the earliest ordering on ties).
"""

import uuid
from collections.abc import Callable, Sequence

import numpy as np

from execution_optimizer.enumerator.ordering_strategy_enum import OrderingStrategyEnum
from execution_optimizer.exceptions import (
    GeometryFailureEnum,
    OptimizationError,
    PatternGeometryError,
)
from execution_optimizer.model.camera_intrinsics_dto import CameraIntrinsicsDTO
from execution_optimizer.model.dispensing_optimizer_config_dto import (
    DispensingOptimizerConfigDTO,
)
from execution_optimizer.model.group_fit_dto import GroupFitDTO
from execution_optimizer.model.pattern_group_dto import PatternGroupDTO
from execution_optimizer.model.pattern_points_dto import PatternPointsDTO
from execution_optimizer.model.pattern_roi_dto import PatternRoiDTO
from execution_optimizer.util.camera_pose import (
    AreaPx,
    fit_group,
    pose_quaternion,
    verify_group,
)
from execution_optimizer.util.geometry import build_pattern_roi

_MAX_LISTED_IDS = 20


def _format_ids(ids: Sequence[str]) -> str:
    """Format offending ids for an error message, truncating long lists."""
    shown = ', '.join(repr(item) for item in ids[:_MAX_LISTED_IDS])
    extra = len(ids) - _MAX_LISTED_IDS
    return f'[{shown}{f", (+{extra} more)" if extra > 0 else ""}]'


def _is_uuid4(value: str) -> bool:
    """True for a canonical hyphenated UUID4 string (case-insensitive)."""
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError):
        return False
    return parsed.version == 4 and str(parsed) == value.lower()


class _GroupFitter:
    """Memoised group feasibility test; the result depends only on the member set."""

    def __init__(
        self,
        rois: Sequence[PatternRoiDTO],
        config: DispensingOptimizerConfigDTO,
        intrinsics: CameraIntrinsicsDTO,
        area_px: AreaPx,
    ) -> None:
        self._rois = rois
        self._config = config
        self._intrinsics = intrinsics
        self._area_px = area_px
        self._cache: dict[tuple[int, ...], GroupFitDTO | None] = {}

    def fit(self, members: Sequence[int]) -> GroupFitDTO | None:
        """Camera placement for the given request indices, or None if infeasible."""
        key = tuple(sorted(members))
        if key not in self._cache:
            self._cache[key] = fit_group(
                [self._rois[index] for index in key],
                self._config,
                self._intrinsics,
                self._area_px,
            )
        return self._cache[key]

    def verify(self, members: Sequence[int], fit: GroupFitDTO) -> bool:
        """Independent re-check of a placement."""
        return verify_group(
            [self._rois[index] for index in sorted(members)],
            fit,
            self._config,
            self._intrinsics,
            self._area_px,
        )


class DispensingGroupingService:
    """Groups dispensing patterns and computes the image capture pose of every group."""

    def __init__(self, config: DispensingOptimizerConfigDTO) -> None:
        """Store the validated configuration.

        :param config: Immutable optimizer configuration.
        :type config: DispensingOptimizerConfigDTO
        """
        self._config = config

    def group(
        self, patterns: Sequence[PatternPointsDTO], intrinsics: CameraIntrinsicsDTO
    ) -> list[PatternGroupDTO]:
        """Group the patterns.

        :param patterns: Patterns with their points in the product frame (metres).
        :param intrinsics: Camera intrinsics.
        :returns: Groups in creation order; pattern ids inside a group are in request order.
        :raises OptimizationError: If the request is invalid or a pattern cannot be imaged.
        """
        self._validate_request(patterns)
        rois = self._build_rois(patterns)
        area_px = self._config.operational_area or (0, 0, intrinsics.width, intrinsics.height)
        fitter = _GroupFitter(rois, self._config, intrinsics, area_px)
        self._check_singletons(rois, fitter)

        best: list[list[int]] | None = None
        for strategy in OrderingStrategyEnum:
            groups = self._eliminate(fitter, self._first_fit(fitter, self._order(rois, strategy)))
            if best is None or len(groups) < len(best):
                best = groups
        assert best is not None
        return self._build_result(rois, fitter, best)

    # region: validation
    def _validate_request(self, patterns: Sequence[PatternPointsDTO]) -> None:
        """Fail the whole request on the first failing validation class."""
        if not patterns:
            raise OptimizationError('No patterns in the request')

        invalid = [pattern.id for pattern in patterns if not _is_uuid4(pattern.id)]
        if invalid:
            raise OptimizationError(f'Invalid pattern ids (not UUID4): {_format_ids(invalid)}')

        seen: set[str] = set()
        duplicates: list[str] = []
        for pattern in patterns:
            key = pattern.id.lower()
            if key in seen and pattern.id not in duplicates:
                duplicates.append(pattern.id)
            seen.add(key)
        if duplicates:
            raise OptimizationError(f'Duplicate pattern ids: {_format_ids(duplicates)}')

        frame = self._config.product_frame_id
        wrong_frame = [
            pattern.id for pattern in patterns
            if any(frame_id != frame for frame_id in pattern.frame_ids)
        ]
        if wrong_frame:
            raise OptimizationError(
                f"Pattern poses not in product frame '{frame}': {_format_ids(wrong_frame)}"
            )

        empty = [pattern.id for pattern in patterns if not pattern.points]
        if empty:
            raise OptimizationError(f'Patterns without points: {_format_ids(empty)}')

    def _build_rois(self, patterns: Sequence[PatternPointsDTO]) -> list[PatternRoiDTO]:
        """Compute all ROIs, collecting every offending pattern id before failing."""
        rois: list[PatternRoiDTO] = []
        failures: dict[GeometryFailureEnum, list[str]] = {}
        tolerance = self._config.planarity_tolerance_m
        for index, pattern in enumerate(patterns):
            try:
                rois.append(build_pattern_roi(pattern, index, tolerance))
            except PatternGeometryError as error:
                failures.setdefault(error.kind, []).append(error.pattern_id)
        messages: list[str] = []
        if GeometryFailureEnum.NON_PLANAR in failures:
            messages.append(
                f'Patterns not planar (max deviation > {self._config.planarity_tolerance_mm} mm): '
                f'{_format_ids(failures[GeometryFailureEnum.NON_PLANAR])}'
            )
        if GeometryFailureEnum.HORIZONTAL_NORMAL in failures:
            messages.append(
                'Patterns with a horizontal surface normal: '
                f'{_format_ids(failures[GeometryFailureEnum.HORIZONTAL_NORMAL])}'
            )
        if messages:
            raise OptimizationError('; '.join(messages))
        return rois

    @staticmethod
    def _check_singletons(rois: Sequence[PatternRoiDTO], fitter: _GroupFitter) -> None:
        """Every pattern must be imageable on its own."""
        infeasible = [roi.id for roi in rois if fitter.fit([roi.index]) is None]
        if infeasible:
            raise OptimizationError(
                'Patterns cannot be captured with the configured criteria (resolution range, '
                f'operational area, camera model): {_format_ids(infeasible)}'
            )
    # endregion: validation

    # region: heuristic
    @staticmethod
    def _order(rois: Sequence[PatternRoiDTO], strategy: OrderingStrategyEnum) -> list[int]:
        """Request indices in the order the given strategy visits them (deterministic)."""
        centroids = [np.mean(np.array(roi.corners), axis=0) for roi in rois]
        keys: dict[OrderingStrategyEnum, Callable[[int], tuple]] = {
            OrderingStrategyEnum.AREA_DESC: lambda i: (-rois[i].area_m2, rois[i].id),
            OrderingStrategyEnum.X_THEN_Y: lambda i: (centroids[i][0], centroids[i][1], i),
            OrderingStrategyEnum.Y_THEN_X: lambda i: (centroids[i][1], centroids[i][0], i),
            OrderingStrategyEnum.DIAGONAL: lambda i: (centroids[i][0] + centroids[i][1], i),
            OrderingStrategyEnum.ANTI_DIAGONAL: lambda i: (centroids[i][0] - centroids[i][1], i),
        }
        return sorted(range(len(rois)), key=keys[strategy])

    @staticmethod
    def _place(fitter: _GroupFitter, groups: list[list[int]], index: int) -> bool:
        """Add the pattern to the first group that stays feasible."""
        for members in groups:
            if fitter.fit(members + [index]) is not None:
                members.append(index)
                return True
        return False

    def _first_fit(self, fitter: _GroupFitter, order: Sequence[int]) -> list[list[int]]:
        """First-fit grouping in the given order (singletons are known to be feasible)."""
        groups: list[list[int]] = []
        for index in order:
            if not self._place(fitter, groups, index):
                groups.append([index])
        return groups

    def _eliminate(self, fitter: _GroupFitter, groups: list[list[int]]) -> list[list[int]]:
        """Remove groups by re-inserting their patterns into the others, smallest first."""
        improved = True
        while improved and len(groups) > 1:
            improved = False
            for victim in sorted(range(len(groups)), key=lambda k: (len(groups[k]), k)):
                trial = [list(members) for k, members in enumerate(groups) if k != victim]
                if all(self._place(fitter, trial, index) for index in groups[victim]):
                    groups = trial
                    improved = True
                    break
        return groups
    # endregion: heuristic

    # region: result
    @staticmethod
    def _build_result(
        rois: Sequence[PatternRoiDTO], fitter: _GroupFitter, groups: list[list[int]]
    ) -> list[PatternGroupDTO]:
        """Build the output groups and check the guarantees on them."""
        result: list[PatternGroupDTO] = []
        for number, members in enumerate(groups):
            members = sorted(members)
            fit = fitter.fit(members)
            if fit is None or not fitter.verify(members, fit):
                raise OptimizationError(f'Internal consistency check failed for group {number}')
            result.append(PatternGroupDTO(
                pattern_ids=tuple(rois[index].id for index in members),
                position=fit.position,
                quaternion_xyzw=pose_quaternion(
                    np.array(fit.forward), np.array(fit.right), np.array(fit.down)
                ),
            ))
        if sorted(index for members in groups for index in members) != list(range(len(rois))):
            raise OptimizationError('Internal consistency check failed: pattern assignment')
        return result
    # endregion: result
