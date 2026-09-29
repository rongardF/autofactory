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
"""Integration tests for MovementController + PlanningCoordinator.

These tests exercise the full stack with a real ROS2 middleware, replacing only
the external move_group process with a minimal ``MockMoveGroupNode`` that provides:

  - Service  ``/move_group/get_planning_scene``  (GetPlanningScene)
  - Service  ``/move_group/plan_sequence_path``   (GetMotionSequence)
  - Service  ``/move_group/plan_kinematic_path``  (GetMotionPlan)
  - Action   ``/move_group/execute_trajectory``   (moveit_msgs/ExecuteTrajectory)

The ``MockMoveGroupNode`` exposes ``planning_success`` / ``execution_success``
boolean flags so individual tests can exercise error-propagation paths without
changing fixtures.

All nodes share a single ``MultiThreadedExecutor`` (8 threads) running in a
daemon background thread.  Goals are sent from the main test thread using
``send_goal_async`` + ``threading.Event`` so executor threads remain free to
process service/action responses (no deadlock risk).
"""

import threading
from typing import Optional

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import ExecuteTrajectory as MoveItExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan, GetMotionSequence, GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint

from movement_controller.action import (
    ExecuteTrajectory,
    ExecuteTrajectory_GetResult_Response as ExecuteTrajectoryResponse,
)
from movement_controller.models.constraint_config_dto import ConstraintConfigDTO
from movement_controller.msg import TrajectoryPath
from movement_controller.movement_controller import MovementController

# ---------------------------------------------------------------------------
# Pydantic forward-reference resolution
#
# PlanResultDTO and PlanningSessionDTO reference moveit_msgs types only inside
# TYPE_CHECKING blocks.  Pydantic v2 cannot resolve those references at runtime
# without an explicit model_rebuild().  Using ``object`` as the namespace value
# is valid: any real MotionSequenceResponse / RobotState instance is a subclass
# of object, so validators accept them.
# ---------------------------------------------------------------------------
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.planning_session_dto import PlanningSessionDTO


PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': object})
PlanningSessionDTO.model_rebuild(_types_namespace={'RobotState': object})

# region: constants for building mock responses
_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
_HOME_POSITIONS = [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]

_PATH_ID_1 = '00000000-0000-4000-8000-000000000001'
_PATH_ID_2 = '00000000-0000-4000-8000-000000000002'
_PATH_ID_3 = '00000000-0000-4000-8000-000000000003'

# endregion: constants for building mock responses


class MockMoveGroupNode(Node):
    """Minimal mock that replaces move_group for integration testing.

    Provides the ROS2 interfaces consumed by MovementController and its
    PlanningCoordinator (PILZ ``plan_sequence_path`` and OMPL
    ``plan_kinematic_path``).  All handlers are synchronous for simplicity.

    Attributes:
        planning_success: When False, plan_sequence/plan_kinematic return an error code.
        execution_success: When False, execute_trajectory aborts the goal.
        scene_request_count: Number of get_planning_scene calls received.
        plan_request_count: Number of plan_sequence_path calls received.
        ompl_request_count: Number of plan_kinematic_path calls received.
        exec_request_count: Number of execute_trajectory goals received.
    """

    def __init__(self) -> None:
        super().__init__('mock_move_group')

        self.planning_success: bool = True
        self.execution_success: bool = True
        self.scene_request_count: int = 0
        self.plan_request_count: int = 0
        self.ompl_request_count: int = 0
        self.exec_request_count: int = 0

        cb = ReentrantCallbackGroup()

        self._scene_srv = self.create_service(
            GetPlanningScene,
            'get_planning_scene',
            self._handle_get_planning_scene,
            callback_group=cb,
        )
        self._plan_srv = self.create_service(
            GetMotionSequence,
            'plan_sequence_path',
            self._handle_plan_sequence,
            callback_group=cb,
        )
        self._ompl_srv = self.create_service(
            GetMotionPlan,
            'plan_kinematic_path',
            self._handle_plan_kinematic,
            callback_group=cb,
        )
        self._exec_action = ActionServer(
            self,
            MoveItExecuteTrajectory,
            'execute_trajectory',
            execute_callback=self._handle_execute_trajectory,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
            callback_group=cb,
        )

    # region: private helpers for building responses
    def _make_robot_state(self) -> RobotState:
        rs = RobotState()
        rs.joint_state.name = _JOINT_NAMES
        rs.joint_state.position = _HOME_POSITIONS
        rs.joint_state.velocity = [0.0] * 6
        return rs

    def _make_trajectory(self) -> RobotTrajectory:
        """Return a minimal valid ``RobotTrajectory`` with one waypoint."""
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = _JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [0.1, -1.5, 1.5, -1.5, -1.5, 0.1]
        pt.velocities = [0.0] * 6
        pt.time_from_start.sec = 1
        traj.joint_trajectory.points = [pt]
        return traj

    # endregion: private helpers for building responses

    # region: service/action handlers
    def _handle_get_planning_scene(
        self,
        request: GetPlanningScene.Request,
        response: GetPlanningScene.Response,
    ) -> GetPlanningScene.Response:
        self.scene_request_count += 1
        response.scene.robot_state = self._make_robot_state()
        return response

    def _handle_plan_sequence(
        self,
        request: GetMotionSequence.Request,
        response: GetMotionSequence.Response,
    ) -> GetMotionSequence.Response:
        self.plan_request_count += 1
        if not self.planning_success:
            response.response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        response.response.error_code.val = MoveItErrorCodes.SUCCESS
        response.response.planned_trajectories = [self._make_trajectory()]
        return response

    def _handle_plan_kinematic(
        self,
        request: GetMotionPlan.Request,
        response: GetMotionPlan.Response,
    ) -> GetMotionPlan.Response:
        self.ompl_request_count += 1
        if not self.planning_success:
            response.motion_plan_response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        response.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
        response.motion_plan_response.trajectory = self._make_trajectory()
        return response

    def _handle_execute_trajectory(
        self, goal_handle: ServerGoalHandle
    ) -> MoveItExecuteTrajectory.Result:
        self.exec_request_count += 1
        result = MoveItExecuteTrajectory.Result()
        if not self.execution_success:
            result.error_code.val = MoveItErrorCodes.FAILURE
            goal_handle.abort()
            return result
        result.error_code.val = MoveItErrorCodes.SUCCESS
        goal_handle.succeed()
        return result

    # endregion: service/action handlers

    # region: test utilities
    def reset(self, *, planning_success: bool = True, execution_success: bool = True) -> None:
        """Reset counters and behaviour flags.  Call at the start of each test."""
        self.scene_request_count = 0
        self.plan_request_count = 0
        self.ompl_request_count = 0
        self.exec_request_count = 0
        self.planning_success = planning_success
        self.execution_success = execution_success

    # endregion: test utilities

# region: pytest fixtures for ROS2 context, executor, and nodes
@pytest.fixture(scope='module')
def ros_context():
    """Initialise rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def executor(ros_context):
    """MultiThreadedExecutor spinning in a daemon background thread.

    8 threads ensures the executor can handle concurrent blocking operations
    (queue.get in iterate_planned_trajectories, send_goal event.wait, service
    handlers) without deadlocking.
    """
    exec_ = MultiThreadedExecutor(num_threads=8)
    thread = threading.Thread(target=exec_.spin, daemon=True)
    thread.start()
    yield exec_
    exec_.shutdown()
    thread.join(timeout=2.0)


@pytest.fixture(scope='module')
def mock_move_group(executor):
    """MockMoveGroupNode added to the shared executor."""
    node = MockMoveGroupNode()
    executor.add_node(node)
    yield node
    executor.remove_node(node)
    node.destroy_node()


@pytest.fixture(scope='module')
def controller(executor, mock_move_group):
    """MovementController configured and activated once for the module.

    ``mock_move_group`` is listed as a dependency to guarantee it is fully
    running before ``on_activate`` calls ``wait_for_service``.
    """
    node = MovementController()
    executor.add_node(node)

    # Initialize STRING_ARRAY / DOUBLE_ARRAY constraint parameters to empty lists.
    # These are declared without defaults in MovementController.__init__ (Phase 5 Plan 01)
    # and raise ParameterUninitializedException if not set before on_configure().
    from rclpy.parameter import Parameter
    node.set_parameters([
        Parameter('constraints.joint.names', Parameter.Type.STRING_ARRAY, []),
        Parameter('constraints.joint.lower_limits', Parameter.Type.DOUBLE_ARRAY, []),
        Parameter('constraints.joint.upper_limits', Parameter.Type.DOUBLE_ARRAY, []),
    ])

    cfg_state = LifecycleState(label='unconfigured', state_id=0)
    assert node.on_configure(cfg_state) == TransitionCallbackReturn.SUCCESS

    act_state = LifecycleState(label='inactive', state_id=1)
    assert node.on_activate(act_state) == TransitionCallbackReturn.SUCCESS

    yield node

    deact_state = LifecycleState(label='active', state_id=2)
    node.on_deactivate(deact_state)
    cleanup_state = LifecycleState(label='inactive', state_id=1)
    node.on_cleanup(cleanup_state)
    executor.remove_node(node)
    node.destroy_node()


@pytest.fixture(scope='module')
def action_client(executor, controller):
    """ActionClient for ``movement_controller/execute_trajectory`` on a helper node."""
    helper = Node('integration_test_client')
    client = ActionClient(helper, ExecuteTrajectory, 'movement_controller/execute_trajectory')
    executor.add_node(helper)
    assert client.wait_for_server(timeout_sec=5.0), (
        'execute_trajectory action server not available within 5 s'
    )
    yield client
    executor.remove_node(helper)
    helper.destroy_node()

# endregion: pytest fixtures for ROS2 context, executor, and nodes

# region: helper functions
def _make_lin_path(path_id: str, blend_radius: float = 0.0) -> TrajectoryPath:
    """Build a fully-populated LIN ``TrajectoryPath`` message."""
    path = TrajectoryPath()
    path.path_id = path_id
    path.motion_type = 'LIN'
    path.blend_radius = blend_radius
    path.target_pose = PoseStamped()
    path.target_pose.header.frame_id = 'base_link'
    path.target_pose.pose.position.x = 0.5
    path.target_pose.pose.position.y = 0.0
    path.target_pose.pose.position.z = 0.5
    path.target_pose.pose.orientation.w = 1.0
    path.circ_point = Point()
    path.cartesian_speed = 0.0
    path.cartesian_acceleration = 0.0
    path.tool_frame = ''
    path.circ_type = 'interim'
    return path


def _send_goal_and_wait(
    client: ActionClient,
    goal: ExecuteTrajectory.Goal,
    timeout: float = 15.0,
) -> Optional[ExecuteTrajectory.Result]:
    """Send a goal from the test thread and block until the result arrives.

    Uses ``send_goal_async`` + ``threading.Event`` so the executor's background
    threads remain free to process all intermediate service/action responses.

    Returns the ``ExecuteTrajectory.Result``,
    or ``None`` if the goal was rejected or the timeout expired.
    """
    done = threading.Event()
    result_holder: list = [None]

    def _on_result(future):
        result_holder[0] = future.result().result
        done.set()

    def _on_goal_response(future):
        handle = future.result()
        if not handle.accepted:
            done.set()
            return
        future2 = handle.get_result_async()
        future2.add_done_callback(_on_result)

    future = client.send_goal_async(goal)
    future.add_done_callback(_on_goal_response)
    done.wait(timeout=timeout)
    return result_holder[0]

# endregion: helper functions

# region: successful execution tests
def test_controller_activates_successfully(controller):
    """After configure + activate, controller resources are fully initialised."""
    assert controller._planner_service is not None
    assert controller._action_server is not None
    assert controller._execute_trajectory_client is not None
    assert controller._is_executing is False


def test_single_lin_path_succeeds(action_client, mock_move_group):
    """Single LIN path → 1 scene request, 1 plan request, 1 exec request → success."""
    mock_move_group.reset()

    goal = ExecuteTrajectory.Goal()
    goal.paths = [_make_lin_path(_PATH_ID_1)]

    response = _send_goal_and_wait(action_client, goal)

    assert response is not None, 'goal timed out or was rejected'
    assert response.success is True
    assert response.error_message == ''
    assert _PATH_ID_1 in response.trajectory_paths_completed
    assert mock_move_group.scene_request_count == 1
    assert mock_move_group.plan_request_count == 1
    assert mock_move_group.exec_request_count == 1


def test_two_separate_paths_each_planned_and_executed(action_client, mock_move_group):
    """Two LIN paths with blend_radius=0.0 form two separate groups.

    Expected: 1 scene request, 2 plan requests (one per group), 2 exec requests.
    """
    mock_move_group.reset()

    goal = ExecuteTrajectory.Goal()
    goal.paths = [
        _make_lin_path(_PATH_ID_1, blend_radius=0.0),
        _make_lin_path(_PATH_ID_2, blend_radius=0.0),
    ]

    response = _send_goal_and_wait(action_client, goal)

    assert response is not None, 'goal timed out or was rejected'
    assert response.success is True
    assert _PATH_ID_1 in response.trajectory_paths_completed
    assert _PATH_ID_2 in response.trajectory_paths_completed
    assert mock_move_group.scene_request_count == 1
    assert mock_move_group.plan_request_count == 2
    assert mock_move_group.exec_request_count == 2


def test_blended_two_paths_one_plan_and_exec(action_client, mock_move_group):
    """Two paths where the first has blend_radius > 0 form a single blend group.

    Expected: 1 scene request, 1 plan request, 1 exec request.
    """
    mock_move_group.reset()

    goal = ExecuteTrajectory.Goal()
    goal.paths = [
        _make_lin_path(_PATH_ID_1, blend_radius=0.05),
        _make_lin_path(_PATH_ID_2, blend_radius=0.0),
    ]

    response = _send_goal_and_wait(action_client, goal)

    assert response is not None, 'goal timed out or was rejected'
    assert response.success is True
    assert mock_move_group.scene_request_count == 1
    assert mock_move_group.plan_request_count == 1
    assert mock_move_group.exec_request_count == 1


def test_three_paths_mixed_blend_groups(action_client, mock_move_group):
    """Three paths: first two blended, third separate → 2 groups.

    Expected: 1 scene request, 2 plan requests, 2 exec requests.
    """
    mock_move_group.reset()

    goal = ExecuteTrajectory.Goal()
    goal.paths = [
        _make_lin_path(_PATH_ID_1, blend_radius=0.05),
        _make_lin_path(_PATH_ID_2, blend_radius=0.0),   # closes blend group
        _make_lin_path(_PATH_ID_3, blend_radius=0.0),   # new separate group
    ]

    response = _send_goal_and_wait(action_client, goal)

    assert response is not None, 'goal timed out or was rejected'
    assert response.success is True
    assert len(response.trajectory_paths_completed) == 3
    assert mock_move_group.plan_request_count == 2
    assert mock_move_group.exec_request_count == 2

# endregion: successful execution tests

# region: error propagation tests
def test_planning_failure_returns_error_and_no_execution(action_client, mock_move_group):
    """When move_group returns a planning error, the controller returns failure
    without sending an execute request."""
    mock_move_group.reset(planning_success=False)

    goal = ExecuteTrajectory.Goal()
    goal.paths = [_make_lin_path(_PATH_ID_1)]

    response = _send_goal_and_wait(action_client, goal)

    assert response is not None, 'goal timed out'
    assert response.success is False
    assert response.error_message != ''
    # Planning failed → execution must never be attempted
    assert mock_move_group.exec_request_count == 0


def test_execution_failure_returns_error(action_client, mock_move_group):
    """When execute_trajectory aborts, the controller returns a failure result."""
    mock_move_group.reset(execution_success=False)

    goal = ExecuteTrajectory.Goal()
    goal.paths = [_make_lin_path(_PATH_ID_1)]

    response = _send_goal_and_wait(action_client, goal)

    assert response is not None, 'goal timed out'
    assert response.success is False
    assert response.error_message != ''
    # Planning must have been attempted even though execution failed
    assert mock_move_group.plan_request_count == 1
    assert mock_move_group.exec_request_count == 1

# endregion: error propagation tests


# region: workspace constraint integration tests
def test_workspace_constraint_violation_causes_planning_failure(
    action_client, mock_move_group, controller
):
    """Workspace constraint active + planning failure → action result success=False.

    Injects a ConstraintConfigDTO(z_max=0.5) directly into the planner service to
    activate a workspace bounding box.  Sets planning_success=False to simulate PILZ
    ValidateSolution rejecting a path that exceeds the workspace bounds.
    Verifies the failure is propagated back to the action client.
    """
    try:
        mock_move_group.reset(planning_success=False)
        controller._planner_service.set_constraints(ConstraintConfigDTO(z_max=0.5))

        path = _make_lin_path(_PATH_ID_1)
        path.target_pose.pose.position.z = 2.0  # exceeds z_max=0.5

        goal = ExecuteTrajectory.Goal()
        goal.paths = [path]

        response = _send_goal_and_wait(action_client, goal)

        assert response is not None, 'goal timed out'
        assert response.success is False
        assert response.error_message != ''
        assert mock_move_group.plan_request_count >= 1
    finally:
        controller._planner_service._constraint_config = None
        mock_move_group.reset()


def test_workspace_constraint_planning_success_when_path_in_bounds(
    action_client, mock_move_group, controller
):
    """Workspace constraint active with generous bounds → planning succeeds.

    Verifies that the constraint infrastructure does not break a valid goal
    when the path is within the workspace bounds and planning succeeds.
    """
    try:
        mock_move_group.reset(planning_success=True)
        controller._planner_service.set_constraints(ConstraintConfigDTO(z_max=2.0))

        path = _make_lin_path(_PATH_ID_1)  # default z=0.5, within z_max=2.0

        goal = ExecuteTrajectory.Goal()
        goal.paths = [path]

        response = _send_goal_and_wait(action_client, goal)

        assert response is not None, 'goal timed out'
        assert response.success is True
    finally:
        controller._planner_service._constraint_config = None
        mock_move_group.reset()

# endregion: workspace constraint integration tests