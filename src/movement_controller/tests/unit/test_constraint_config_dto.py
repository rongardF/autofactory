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
"""Unit tests for ConstraintConfigDTO and PilzPlannerService constraint-building methods."""

from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError
from rclpy.lifecycle import LifecycleNode
from rclpy.impl.rcutils_logger import RcutilsLogger
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive

from movement_controller.models.constraint_config_dto import ConstraintConfigDTO
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.planning_session_dto import PlanningSessionDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService

# Resolve TYPE_CHECKING-only forward references so Pydantic can instantiate these models in tests.
PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': MagicMock})
PlanningSessionDTO.model_rebuild(_types_namespace={'RobotState': MagicMock})


def _make_service(dto: ConstraintConfigDTO | None = None) -> PilzPlannerService:
    """Create a minimal PilzPlannerService backed by a mock node.

    Does NOT call on_activate() since constraint-building methods only need
    _constraint_config to be set — no ROS2 service clients required.
    """
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    if dto is not None:
        svc.set_constraints(dto)
    return svc


# region: ConstraintConfigDTO property tests
def test_workspace_enabled_false_at_sentinel_defaults():
    """Default ConstraintConfigDTO has all axes at sentinel range → workspace_enabled is False."""
    dto = ConstraintConfigDTO()
    assert dto.workspace_enabled is False


def test_workspace_enabled_true_when_z_narrowed():
    """Narrowing z axis below sentinel range → workspace_enabled is True."""
    dto = ConstraintConfigDTO(z_min=0.0, z_max=0.5)
    assert dto.workspace_enabled is True


def test_workspace_enabled_true_when_x_narrowed():
    """Narrowing x axis → workspace_enabled is True."""
    dto = ConstraintConfigDTO(x_min=-2.0, x_max=2.0)
    assert dto.workspace_enabled is True


def test_workspace_disabled_requires_all_axes_at_sentinel():
    """Narrowing only y → workspace_enabled is True (not all axes at sentinel)."""
    dto = ConstraintConfigDTO(y_min=0.0, y_max=1.0)
    assert dto.workspace_enabled is True


def test_orientation_constraint_enabled_false_at_defaults():
    """Default tolerances of 2π → orientation_constraint_enabled is False."""
    dto = ConstraintConfigDTO()
    assert dto.orientation_constraint_enabled is False


def test_orientation_constraint_enabled_true_when_narrowed():
    """Any tolerance tighter than 2π → orientation_constraint_enabled is True."""
    dto = ConstraintConfigDTO(orientation_tolerance_z=0.1)
    assert dto.orientation_constraint_enabled is True

# endregion: ConstraintConfigDTO property tests

# region: ConstraintConfigDTO validation tests
def test_validation_error_x_min_greater_than_x_max():
    """x_min > x_max raises ValidationError."""
    with pytest.raises(ValidationError):
        ConstraintConfigDTO(x_min=1.0, x_max=0.0)


def test_validation_error_y_min_greater_than_y_max():
    """y_min > y_max raises ValidationError."""
    with pytest.raises(ValidationError):
        ConstraintConfigDTO(y_min=2.0, y_max=1.0)


def test_validation_error_z_min_greater_than_z_max():
    """z_min > z_max raises ValidationError."""
    with pytest.raises(ValidationError):
        ConstraintConfigDTO(z_min=1.0, z_max=0.5)

# endregion: ConstraintConfigDTO validation tests

# region: _build_path_constraints tests
def test_build_path_constraints_returns_empty_when_no_constraint_config():
    """No constraint config set → _build_path_constraints returns empty Constraints."""
    svc = _make_service()
    c = svc._build_path_constraints('tool0')
    assert isinstance(c, Constraints)
    assert len(c.position_constraints) == 0
    assert len(c.joint_constraints) == 0
    assert len(c.orientation_constraints) == 0


def test_build_path_constraints_returns_empty_when_all_disabled():
    """All-sentinel ConstraintConfigDTO → no constraints populated in result."""
    svc = _make_service(ConstraintConfigDTO())
    c = svc._build_path_constraints('tool0')
    assert len(c.position_constraints) == 0
    assert len(c.joint_constraints) == 0
    assert len(c.orientation_constraints) == 0


def test_build_path_constraints_box_full_lengths():
    """Workspace BOX uses full axis lengths, not half-extents; center is midpoint."""
    dto = ConstraintConfigDTO(
        x_min=-1.0, x_max=2.0,
        y_min=0.0, y_max=3.0,
        z_min=0.0, z_max=0.5,
    )
    svc = _make_service(dto)
    c = svc._build_path_constraints('tool0')

    assert len(c.position_constraints) == 1
    pc = c.position_constraints[0]  # type: ignore
    assert pc.constraint_region.primitives[0].type == SolidPrimitive.BOX
    # full lengths: (2.0 - -1.0)=3.0, (3.0 - 0.0)=3.0, (0.5 - 0.0)=0.5
    dims = pc.constraint_region.primitives[0].dimensions
    assert dims[0] == pytest.approx(3.0)
    assert dims[1] == pytest.approx(3.0)
    assert dims[2] == pytest.approx(0.5)
    # center midpoints: (-1.0+2.0)/2=0.5, (0.0+3.0)/2=1.5, (0.0+0.5)/2=0.25
    center = pc.constraint_region.primitive_poses[0]
    assert center.position.x == pytest.approx(0.5)
    assert center.position.y == pytest.approx(1.5)
    assert center.position.z == pytest.approx(0.25)
    assert center.orientation.w == pytest.approx(1.0)


def test_build_path_constraints_box_link_name():
    """BOX position constraint uses the provided tool_frame as link_name."""
    dto = ConstraintConfigDTO(z_min=0.0, z_max=0.5)
    svc = _make_service(dto)
    c = svc._build_path_constraints('my_tool')
    assert c.position_constraints[0].link_name == 'my_tool'


def test_build_path_constraints_orientation_fields():
    """Orientation constraint populates all tolerance, parameterization, and link fields."""
    dto = ConstraintConfigDTO(
        orientation_tolerance_x=0.1,
        orientation_tolerance_y=0.2,
        orientation_tolerance_z=0.3,
    )
    svc = _make_service(dto)
    c = svc._build_path_constraints('my_tool')

    assert len(c.orientation_constraints) == 1
    oc = c.orientation_constraints[0]  # type: ignore
    assert oc.link_name == 'my_tool'
    assert oc.absolute_x_axis_tolerance == pytest.approx(0.1)
    assert oc.absolute_y_axis_tolerance == pytest.approx(0.2)
    assert oc.absolute_z_axis_tolerance == pytest.approx(0.3)
    assert oc.parameterization == 0
    assert oc.orientation.w == pytest.approx(1.0)

# endregion: _build_path_constraints tests

# region: _merge_circ_and_path_constraints tests
def _make_arc_constraints(name: str) -> Constraints:
    """Build a minimal CIRC arc Constraints (mimics _build_circ_constraints output)."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import BoundingVolume

    c = Constraints()
    c.name = name
    pos = PositionConstraint()
    pos.link_name = 'tool0'
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.001]
    bv = BoundingVolume()
    bv.primitives = [sphere]
    bv.primitive_poses = [Pose()]
    pos.constraint_region = bv
    c.position_constraints = [pos]
    return c


def _make_box_path_constraints() -> Constraints:
    """Build minimal path Constraints with one BOX position constraint."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import BoundingVolume

    c = Constraints()
    pos = PositionConstraint()
    pos.link_name = 'tool0'
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [1.0, 1.0, 1.0]
    bv = BoundingVolume()
    bv.primitives = [box]
    bv.primitive_poses = [Pose()]
    pos.constraint_region = bv
    c.position_constraints = [pos]
    return c


def test_merge_preserves_circ_arc_at_index_zero():
    """Merged constraints have arc point at [0] and workspace BOX at [1]."""
    svc = _make_service()
    circ = _make_arc_constraints('interim')
    path = _make_box_path_constraints()

    merged = svc._merge_circ_and_path_constraints(circ, path)

    assert merged.name == 'interim'
    assert len(merged.position_constraints) == 2
    # arc point preserved at index 0
    assert merged.position_constraints[0] is circ.position_constraints[0]  # type: ignore
    # BOX appended at index 1
    assert merged.position_constraints[1] is path.position_constraints[0]  # type: ignore


def test_merge_with_center_name():
    """Merged constraints preserve 'center' name from CIRC constraints."""
    svc = _make_service()
    circ = _make_arc_constraints('center')
    merged = svc._merge_circ_and_path_constraints(circ, Constraints())
    assert merged.name == 'center'


def test_merge_with_empty_path_constraints():
    """Merging CIRC with empty path constraints yields only arc point (no BOX added)."""
    svc = _make_service()
    circ = _make_arc_constraints('center')
    path = Constraints()  # all empty

    merged = svc._merge_circ_and_path_constraints(circ, path)

    assert merged.name == 'center'
    assert len(merged.position_constraints) == 1
    assert merged.position_constraints[0] is circ.position_constraints[0]  # type: ignore


def test_merge_propagates_joint_and_orientation_from_path():
    """Joint and orientation constraints come from path, not circ."""
    from moveit_msgs.msg import OrientationConstraint

    svc = _make_service()
    circ = _make_arc_constraints('interim')
    path = Constraints()
    jc = JointConstraint()
    jc.joint_name = 'j1'
    path.joint_constraints = [jc]
    oc = OrientationConstraint()
    oc.link_name = 'tool0'
    path.orientation_constraints = [oc]

    merged = svc._merge_circ_and_path_constraints(circ, path)

    assert len(merged.joint_constraints) == 1
    assert merged.joint_constraints[0] is jc  # type: ignore
    assert len(merged.orientation_constraints) == 1
    assert merged.orientation_constraints[0] is oc  # type: ignore

# endregion: _merge_circ_and_path_constraints tests
