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
"""BasePlannerService — shared constraint/scaling helpers for planner services."""

from __future__ import annotations

from rclpy.lifecycle.node import LifecycleNode
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionSequenceResponse,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models import (
    ConstraintConfigDTO,
    TrajectoryPathDTO,
)


class BasePlannerService:
    """Shared base for planner services.

    Holds the parent node reference, planning group name, and active constraint
    configuration, and provides the constraint-building, start-state validation,
    scaling-factor, and end-state extraction helpers that are common to both the
    PILZ (LIN/CIRC) and OMPL (PTP) planner services.
    """

    def __init__(self, node: LifecycleNode, moveit_group_name: str) -> None:
        """Initialise shared planner state.

        :param node: Parent lifecycle node; used to access the logger and to
            create ROS 2 service clients in subclasses.
        :type node: LifecycleNode
        :param moveit_group_name: MoveIt 2 planning group name as defined in
            the SRDF (e.g. ``'ur_manipulator'``).
        :type moveit_group_name: str
        """
        self._group_name = moveit_group_name
        self._node = node
        self._logger = node.get_logger()
        self._constraint_config: ConstraintConfigDTO | None = None
        self._callback_group = ReentrantCallbackGroup()

    # region: shared helpers
    def _build_pose_goal_constraints(self, link_name: str, pose_stamped) -> Constraints:
        """Build goal Constraints from a PoseStamped (replaces C++ constructGoalConstraints)."""
        constraints = Constraints()
        constraints.name = f'goal_constraints_{link_name}'

        pos = PositionConstraint()
        pos.header = pose_stamped.header
        pos.link_name = link_name
        pos.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.0001]
        target = Pose()
        target.position = pose_stamped.pose.position
        target.orientation.w = 1.0
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target]
        pos.constraint_region = bv
        constraints.position_constraints = [pos]

        ori = OrientationConstraint()
        ori.header = pose_stamped.header
        ori.link_name = link_name
        ori.orientation = pose_stamped.pose.orientation
        ori.absolute_x_axis_tolerance = 0.001
        ori.absolute_y_axis_tolerance = 0.001
        ori.absolute_z_axis_tolerance = 0.001
        ori.weight = 1.0
        constraints.orientation_constraints = [ori]

        self._logger.debug(f'Constructed goal constraints for link {link_name}:\n{constraints}')

        return constraints

    def _build_path_constraints(self, tool_frame: str) -> Constraints:
        """Build path Constraints from the active constraint config for a given tool frame."""
        constraints = Constraints()
        constraints.name = f'path_constraints_{tool_frame}'
        if self._constraint_config is None:
            return constraints
        self._logger.debug(f'Building path constraints')
        cfg = self._constraint_config

        # Workspace BOX (when workspace bounds are tighter than sentinel range)
        if cfg.workspace_enabled:
            self._logger.debug(
                f'Building workspace constraints with bounds x: [{cfg.x_min}, {cfg.x_max}], '
                f'y: [{cfg.y_min}, {cfg.y_max}], z: [{cfg.z_min}, {cfg.z_max}]'
            )  # FIXME: we should use workspace constraint instead
            pos = PositionConstraint()
            pos.header.frame_id = 'base_link'
            pos.link_name = tool_frame
            pos.weight = 1.0
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [cfg.x_max - cfg.x_min, cfg.y_max - cfg.y_min, cfg.z_max - cfg.z_min]
            center = Pose()
            center.position.x = (cfg.x_max + cfg.x_min) / 2.0
            center.position.y = (cfg.y_max + cfg.y_min) / 2.0
            center.position.z = (cfg.z_max + cfg.z_min) / 2.0
            center.orientation.w = 1.0
            bv = BoundingVolume()
            bv.primitives = [box]
            bv.primitive_poses = [center]
            pos.constraint_region = bv
            constraints.position_constraints.append(pos)  # type: ignore
            self._logger.debug(f'Constructed workspace constraints:\n{constraints.position_constraints}')

        # Orientation constraint (identity quaternion as neutral reference)
        if cfg.orientation_constraint_enabled:
            self._logger.debug(
                f'Building orientation constraints with tolerances x: {cfg.orientation_tolerance_x}, '
                f'y: {cfg.orientation_tolerance_y}, z: {cfg.orientation_tolerance_z} for tool frame {tool_frame}'
            )
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = tool_frame
            oc.orientation.w = 1.0
            oc.absolute_x_axis_tolerance = cfg.orientation_tolerance_x
            oc.absolute_y_axis_tolerance = cfg.orientation_tolerance_y
            oc.absolute_z_axis_tolerance = cfg.orientation_tolerance_z
            oc.parameterization = 0
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)  # type: ignore
            self._logger.debug(f'Constructed orientation constraints:\n{constraints.orientation_constraints}')

        self._logger.debug(f'Final constructed path constraints:\n{constraints}')
        return constraints

    def _extract_end_state(self, motion_sequence_response: MotionSequenceResponse) -> RobotState:
        """Extract the final joint state from the last planned trajectory (D-08)."""
        trajectories: list[RobotTrajectory] = motion_sequence_response.planned_trajectories  # type: ignore
        self._logger.debug(f'Extracting end state from last of {len(trajectories)} planned trajectory/trajectories')
        last_traj = trajectories[-1]
        jt = last_traj.joint_trajectory
        last_point = jt.points[-1]  # type: ignore
        state = RobotState()
        state.joint_state.name = list(jt.joint_names)
        state.joint_state.position = list(last_point.positions)
        state.joint_state.velocity = [0.0] * len(jt.joint_names)
        self._logger.debug(
            f'Extracted end state for joints {list(jt.joint_names)}: '
            f'positions={list(last_point.positions)}'
        )
        return state

    def _start_state_invalid(self, start_state: RobotState) -> bool:
        """Check if the start state violates any active constraints """
        if self._constraint_config is None:
            return False

        self._logger.debug(f'Checking if start state is valid:\n{start_state}')

        cfg = self._constraint_config

        # Check workspace bounds
        if cfg.workspace_enabled:
            self._logger.debug('Checking workspace bounds violations')
            # TODO: to check workspace bounds

        # Check orientation constraints
        if cfg.orientation_constraint_enabled:
            self._logger.debug('Checking orientation constraints violations')
            # TODO: to check orientation constraints

        return False

    def _calculate_scaling_factors(self, path_dto: TrajectoryPathDTO) -> tuple[float, float]:
        """Calculate velocity and acceleration scaling factors based on cartesian speed limit for LIN/CIRC paths."""
        if self._constraint_config is None:
            self._logger.warning('Constraint config not set; using default scaling factors of 1.0')
            return 1.0, 1.0

        if path_dto.motion_type in [MotionTypeEnum.LIN, MotionTypeEnum.CIRC]:
            if self._constraint_config.max_cartesian_speed > 0.0:
                self._logger.debug("Max cartesian speed is set, calculating scaling factor")
                vel_scaling_factor = min(
                    1.0,
                    path_dto.cartesian_speed / self._constraint_config.max_cartesian_speed
                )
            else:
                vel_scaling_factor = 1.0

            if self._constraint_config.max_cartesian_acceleration > 0.0:
                self._logger.debug("Max cartesian acceleration is set, calculating scaling factor")
                acc_scaling_factor = min(
                    1.0,
                    path_dto.cartesian_acceleration / self._constraint_config.max_cartesian_acceleration
                )
            else:
                acc_scaling_factor = 1.0

            self._logger.debug(
                f'Calculated scaling factors for path {path_dto.path_id} with cartesian speed {path_dto.cartesian_speed}: '
                f'velocity_scaling_factor={vel_scaling_factor}, acceleration_scaling_factor={acc_scaling_factor}'
            )
            return vel_scaling_factor, acc_scaling_factor
        else:
            if self._constraint_config.max_joint_speed > 0.0:
                self._logger.debug("Max joint speed is set, calculating scaling factor")
                vel_scaling_factor = min(
                    1.0,
                    path_dto.joint_speed / self._constraint_config.max_joint_speed
                )
            else:
                vel_scaling_factor = 1.0

            if self._constraint_config.max_joint_acceleration > 0.0:
                self._logger.debug("Max joint acceleration is set, calculating scaling factor")
                acc_scaling_factor = min(
                    1.0,
                    path_dto.joint_acceleration / self._constraint_config.max_joint_acceleration
                )
            else:
                acc_scaling_factor = 1.0

            self._logger.debug(
                f'Calculated scaling factors for path {path_dto.path_id} with joint speed {path_dto.joint_speed}: '
                f'velocity_scaling_factor={vel_scaling_factor}, acceleration_scaling_factor={acc_scaling_factor}'
            )
            return vel_scaling_factor, acc_scaling_factor

    # endregion: shared helpers

    # region: shared public methods
    def set_constraints(self, dto: ConstraintConfigDTO) -> None:
        """Store the active constraint configuration.

        Must be called before planning so that path constraints and
        speed/acceleration scaling factors are derived correctly for each
        trajectory path.

        :param dto: Validated constraint configuration to apply.
        :type dto: ConstraintConfigDTO
        """
        self._logger.debug(
            f'Storing constraint configuration: workspace_enabled={dto.workspace_enabled}, '
            f'orientation_constraint_enabled={dto.orientation_constraint_enabled}, '
            f'max_cartesian_speed={dto.max_cartesian_speed}, max_joint_speed={dto.max_joint_speed}'
        )
        self._constraint_config = dto

    # endregion: shared public methods
