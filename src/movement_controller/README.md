# movement_controller

ROS 2 Jazzy package that provides a vendor-agnostic action interface for executing
collision-aware, optionally blended multi-path trajectories on industrial robot
arms using MoveIt 2.

Motion is planned by two complementary MoveIt planners behind a single action:

- **PILZ** industrial motion planner for `LIN` and `CIRC` cartesian segments
  (with optional blending between consecutive paths).
- **OMPL** for collision-aware `PTP` (point-to-point) motion.

A `PlanningCoordinator` routes each trajectory group to the correct planner,
threads the end state of one group into the start state of the next, and runs
**look-ahead planning** asynchronously so the next group is planned while the
current one executes.

Launch support currently targets Universal Robots (UR), in both **Gazebo
simulation** and on **real hardware**, but the launch layer is structured so
additional robot families can be added without touching the node code.

## Highlights

- Lifecycle-based controller node (`rclpy.lifecycle.LifecycleNode`)
- Single action API for ordered, mixed-motion trajectory execution:
	- Action: `movement_controller/execute_trajectory`
	- Type: `movement_controller/action/ExecuteTrajectory`
- LIN / PTP / CIRC path segments via `TrajectoryPath.msg`
- Dual-planner routing: PILZ (LIN/CIRC) + OMPL (collision-aware PTP)
- Asynchronous look-ahead planning pipelined with execution
- Optional path blending (grouped execution based on blend radius)
- Goal-level constraint validation (workspace box, orientation tolerances,
	speed/acceleration caps) with per-joint C-space limits enforced globally
- Goal cancellation support (including the MoveIt cancellation workaround topic)
- Sim/real branching handled entirely at the launch layer (`simulated` arg)

## Package Layout

```text
src/movement_controller/
|- action/ExecuteTrajectory.action        # action definition
|- msg/TrajectoryPath.msg                 # per-segment path message
|- movement_controller/
|  |- movement_controller.py              # MovementController lifecycle node
|  |- models/                             # Pydantic DTOs (goal, path, constraints, plan, session)
|  |- services/                           # PlanningCoordinator + PILZ/OMPL/base planner services
|  |- enums/                              # MotionType / CircType / FeedbackStatus enums
|  |- exceptions/                         # domain exceptions
|  `- utils/                              # TrajectoryGrouper
|- launch/
|  |- launch.py                           # generic dispatcher (vendor/sim resolution)
|  `- ur_launch/                          # UR vendor launches
|     |- gz_sim.launch.py                 # UR in Gazebo simulation
|     `- hardware.launch.py               # UR on real hardware
|- config/
|  |- joint_limits.yaml                   # velocity/accel + position (C-space) limits
|  |- pilz_cartesian_limits.yaml          # cartesian speed/accel caps
|  |- kinematics.yaml
|  |- moveit_controllers.yaml
|  |- ompl_planning.yaml
|  |- pilz_industrial_motion_planner_planning.yaml
|  |- moveit.rviz
|  `- ur/                                 # UR controllers + initial positions
|- srdf/ur/                               # UR semantic description (xacro)
|- urdf/ur/                               # UR description (xacro)
|- world/default.world                    # default Gazebo world
|- scripts/movement_controller            # installed console entry point
`- tests/
	 |- unit/
	 `- integration/
```

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2 for Jazzy (with PILZ and OMPL planning pipelines)
- Gazebo Harmonic + `ros_gz_sim` / `ros_gz_bridge` / `gz_ros2_control` (simulation)
- UR ROS 2 driver (`ur_robot_driver`) for UR hardware/sim
- Python 3.11+

If you are working in this repository's devcontainer, these are mostly already
provided. Use the project virtual environment and overlay setup shown below.

## Build

To run RViz/Gazebo inside the devcontainer, enable XHost forwarding first:

```bash
sudo -i xhost +local:
```

From workspace root:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

The generic launch file resolves the robot family from `model`, then includes the
matching vendor launch (`gz_sim.launch.py` when `simulated:=true`,
`hardware.launch.py` when `simulated:=false`) alongside `move_group`, RViz, and
this node.

Launch in simulation (default):

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch movement_controller launch.py \
	model:=ur10 \
	simulated:=true \
	rviz:=true \
	debug:=false
```

Launch against real hardware:

```bash
ros2 launch movement_controller launch.py \
	model:=ur10 \
	simulated:=false \
	ip_address:=192.168.1.9 \
	rviz:=true
```

Common launch arguments:

- `model` (default `ur10`) — robot model; the vendor family is derived from it
	(supported UR models: `ur3/ur5/ur10`, `ur3e/ur5e/ur7e/ur10e/ur12e/ur16e`,
	`ur8long`, `ur15/ur18/ur20/ur30`)
- `simulated` (`true`/`false`, default `true`) — Gazebo sim vs. real hardware;
	selects the vendor launch file and drives `use_sim_time`
- `ip_address` (default `192.168.1.9`) — robot controller IP (real hardware)
- `rviz` (`true`/`false`, default `true`)
- `debug` (`true`/`false`, default `false`) — sets node log level to DEBUG
- `world_file` — Gazebo world (default `world/default.world`, simulation only)
- `urdf_file` / `srdf_file` — robot description / semantic xacro paths
- `gazebo_gui` (`true`/`false`, default `true`) — start Gazebo with GUI
- `gz_resource_path` — extra Gazebo resource path

> Note: `use_sim_time` is not a user argument — it is derived from `simulated`.

### Lifecycle Activation

The controller is a lifecycle node. After launch, configure and activate it:

```bash
ros2 lifecycle set /movement_controller configure
ros2 lifecycle set /movement_controller activate
```

Check state:

```bash
ros2 lifecycle get /movement_controller
```

`on_configure` reads and validates all constraint/OMPL parameters and builds the
`PlanningCoordinator`; `on_activate` waits for the PILZ `plan_sequence_path`
service and creates the action server plus the MoveIt `execute_trajectory`
action client.

## Action Interface

### Goal

Action: `movement_controller/execute_trajectory`
Type: `movement_controller/action/ExecuteTrajectory`

Goal field:

- `paths`: ordered list of `movement_controller/msg/TrajectoryPath`

### Result

- `success` (`bool`)
- `error_message` (`string`)
- `trajectory_paths_completed` (`string[]`)

### Feedback

- `status` (`string`)
- `trajectory_path_ids` (`string[]`)

Status values published per execution group (see `FeedbackStatusEnum`):

- `executing` — emitted when a group starts executing
- `completed` — emitted when a group finishes

## TrajectoryPath Message

`TrajectoryPath.msg` fields (per path segment):

- `path_id` — UUID string expected
- `motion_type` — `LIN`, `PTP`, or `CIRC`
- `target_pose` — `geometry_msgs/PoseStamped`
- `blend_radius` (default `0.0`) — blend into the next path (LIN/CIRC groups)
- `cartesian_speed` (default `0.0`) — cartesian speed for the selected link
- `cartesian_acceleration` (default `0.0`) — cartesian acceleration for the selected link
- `joint_speed` (default `0.0`) — joint speed (same value applies to all joints)
- `joint_acceleration` (default `0.0`) — joint acceleration (same value applies to all joints)
- `tool_frame`
- `circ_type` — `interim` (waypoint on arc) or `center` (arc center), CIRC only
- `circ_point` — `geometry_msgs/Point`, CIRC only

The message also exposes string constants (`MOTION_TYPE_*`, `CIRC_TYPE_*`) for
convenience.

## Planning & Execution Model

- **Grouping** — `TrajectoryGrouper` splits the ordered paths into execution
	groups. PTP paths are always isolated into single-path groups; consecutive
	LIN/CIRC paths are grouped for blending based on `blend_radius`.
- **Routing** — `PlanningCoordinator` sends PTP groups to `OmplPlannerService`
	(collision-aware) and LIN/CIRC groups to `PilzPlannerService`
	(`plan_sequence_path`).
- **State chaining** — the live planning scene seeds the first group; each
	successful group's end state becomes the next group's start state.
- **Look-ahead** — planning runs asynchronously on a background thread and
	results are consumed via an iterator, so the next group can be planned while
	the current one is executing.
- **Execution** — each planned trajectory is sent to the MoveIt
	`execute_trajectory` action client and awaited with cancellation support.

## Quick Client Example

You can use the repository sandbox scripts from workspace root, for example:

```bash
python examples/linear_movement_with_constant_speed_and_blending.py
python examples/ptp_movement.py
python examples/circ_movement.py
```

These scripts submit `ExecuteTrajectory` goals to
`movement_controller/execute_trajectory` and print feedback/results. See the
`examples/` directory for more complete client patterns.

## Parameters

The node declares and validates these parameters:

- `moveit_group_name` (default `ur_manipulator`) — MoveIt 2 planning group
- `moveit_connection_timeout` (default `10.0`) — seconds to wait for the PILZ
	`plan_sequence_path` service during `on_activate`
- Workspace bounding box (metres; `±1e9` sentinels = unconstrained):
	- `constraints.workspace.x_min`, `x_max`
	- `constraints.workspace.y_min`, `y_max`
	- `constraints.workspace.z_min`, `z_max`
- Orientation tolerances (radians; default `2π` = unconstrained):
	- `constraints.orientation.tolerance_x`
	- `constraints.orientation.tolerance_y`
	- `constraints.orientation.tolerance_z`
- Speed/acceleration caps (0..1 ratio; `0.0` = unconstrained). Goals whose paths
	exceed any cap are rejected:
	- `constraints.max_cartesian_speed`
	- `constraints.max_cartesian_acceleration`
	- `constraints.max_joint_speed`
	- `constraints.max_joint_acceleration`
- OMPL tuning (collision-aware PTP):
	- `ompl_planning_time` (default `5.0`) — `allowed_planning_time` (s)
	- `ompl_planning_attempts` (default `10`) — `num_planning_attempts`

Default speed/acceleration cap values are computed by the launch setup from the
strictest joint limits in `config/joint_limits.yaml` and the cartesian limits in
`config/pilz_cartesian_limits.yaml`, then passed to the node.

Per-joint position envelopes are enforced globally as C-space bounds: the launch
setup reads `config/joint_limits.yaml` and passes the
`robot_description_planning.joint_limits.<joint>.{min_position,max_position}`
values to `move_group`, so every planner (OMPL, PILZ) respects them without
adding per-request joint path constraints.

## Speed and Acceleration Notes

- Joint speed/acceleration are constrained by the strictest joint limits used in
	configuration.
- Cartesian speed behaviour depends on both translational and rotational
	components. If rotational limits are tight, observed translation speed may be
	lower than requested on mixed-rotation motions.
- MoveIt PILZ interfaces expose acceleration but not a separate deceleration
	setting in this flow; plan accordingly.

## Testing

Run package tests:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select movement_controller
colcon test-result --verbose
```

You can also run pytest directly for quick iteration:

```bash
python -m pytest src/movement_controller/tests/unit -v
python -m pytest src/movement_controller/tests/integration -v
```

## Troubleshooting

- Action server unavailable:
	- Ensure the lifecycle node is `active`.
	- Verify `move_group` and the UR driver (or Gazebo sim) launched successfully.
- Planning service unavailable:
	- Confirm the PILZ sequence service exists:
		`ros2 service list | grep plan_sequence_path`
	- Ensure `move_group` runs the `pilz_industrial_motion_planner` and `ompl`
		pipelines (configured by the launch file).
- No motion / immediate rejection:
	- Check path IDs are valid UUID strings.
	- Check `motion_type` values are valid (`LIN`, `PTP`, `CIRC`).
	- Validate goal values against configured workspace/orientation/speed
		constraints and joint C-space limits.
- Collision-aware PTP fails to plan:
	- Increase `ompl_planning_time` and/or `ompl_planning_attempts`.
- Cancellation behaviour:
	- The implementation includes a MoveIt workaround by publishing `"stop"` to
		`/trajectory_execution_event` due to known MoveIt cancellation limitations
		(see https://github.com/moveit/moveit2/issues/2808).

## License

BSD-3-Clause

