# pattern_executors

ROS 2 package that executes manufacturing *patterns* on a robot arm. A
**pattern** is a sequence of robot movements, each optionally carrying end-tool
*actions* (open/close a dispenser valve, wait). Each manufacturing operation
type gets its own executor node, but all share one abstract contract.

This package provides:

- **`PatternExecutor`** — an abstract `LifecycleNode` base that owns the shared
  behaviour: lifecycle, the `movement_controller` action client, the latched
  `~/executing_patterns` status topic, the end-tool lifecycle manager, the
  single-goal concurrency guard, and the common abort/cancel/error flow.
- **`DispensingPatternExecutor`** — the concrete node for volumetric dispensing:
  valve open/close/wait actions, flow-rate handling, and dispensing metrics.

## Architecture

```
ExecuteDispensingPattern goal
        │
        ▼
DispensingPatternExecutor ──▶ movement_controller/execute_trajectory (action client)
        │                         ▲
        │ feedback-timed          │ executing / completed feedback
        │ valve + wait actions    │
        ▼                         │
volumetric dispenser end tool  (lifecycle + ~/dispense_start, ~/dispense_stop)
```

The executor **does not** parse assembly JSON, inject planning-scene/world
models, mount the physical end tool, or publish the product origin frame. An
upstream orchestrator does those and calls this node's action with already-built
messages.

## Action contract

`DispensingPatternExecutor` serves the `~/execute_dispensing_pattern` action
(`pattern_executors/action/ExecuteDispensingPattern`):

- **Goal**: `DispensingPattern[] patterns`, `string frame_id` (frame all poses
  are expressed in), `string operation_id`.
- **Result**: `bool success`, `string error_message`, `float64 execution_time_s`,
  `string operation_id`, `DispensingPatternMetric[] metrics` (one per executed
  pattern; only fully-completed patterns are reported on failure/cancel).
- **Feedback**: emitted after each completed pattern — `string pattern_id`,
  `float64 execution_time_s`, `DispensingPatternMetric metric`.

Goals are **rejected** when the node is not active or a goal is already
executing (single-goal concurrency). Goals are also rejected when invalid: empty
`patterns`, an unknown `PatternAction.type`/`timing`, or a `wait` action on a
non-zero-length movement.

### Pattern execution

For each pattern the executor:

1. Reconfigures the end-tool flow rate when `flowrate_m3_s` differs from the
   dispenser's current `flowrate` (compared in cc/s to 6 decimals). Reconfigure
   cycle: deactivate → cleanup → set `flowrate` → configure → activate.
2. Runs a valve-closed `PTP` approach to `pattern.start_pose` (no actions).
3. Splits `movements[]` into execution batches, cutting **after** any movement
   that carries a blocking `wait` action. Each batch is one `ExecuteTrajectory`
   goal, so blending is preserved within a batch and a blocking wait holds the
   robot in place between batches.
4. Fires each movement's actions from movement feedback: `timing=start` actions
   on `executing` feedback, `timing=end` actions on `completed` feedback, in
   listed order. Start actions for a movement always complete (including a
   blocking `wait`) before that movement's end actions run.

Action semantics:

| Action | Mapped to | Blocking |
|--------|-----------|----------|
| `open_valve` | `~/dispense_start` (`std_srvs/Trigger`) | No (service round-trip) |
| `close_valve` | `~/dispense_stop` (`endtools/StopDispensing`) | No (service round-trip) |
| `wait` | sleep `wait_time_s` | Yes (no movement in flight) |

Each `open_valve` → `close_valve` cycle yields one `DispensedElementMetric`
(`element_id` = `path_id` of the movement whose `open_valve` started it, volume
converted cc→m³).

### Abort / cancel

The goal is abortable. On cancel request, movement failure, end-tool failure, or
any exception mid-goal the executor cancels the in-flight movement goal, forces
`~/dispense_stop` (so dispensing cannot leak), publishes
`executing_patterns=false`, logs the error, and returns a failure result
(`canceled()` on explicit cancel, otherwise `abort()`).

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tool_sn` | string | `""` (required) | End-tool serial number. Movement `tool_frame` is `{tool_sn}_calibrated_tcp`. |
| `endtool_node_name` | string | `""` (required) | Node name of the end tool whose lifecycle is driven. |
| `movement_action_name` | string | `movement_controller/execute_trajectory` | Movement controller action name. |
| `movement_server_timeout_s` | double | `5.0` | `wait_for_server` timeout. |
| `endtool_service_timeout_s` | double | `5.0` | End-tool lifecycle/parameter/service call timeout. |
| `simulated` | bool | `true` | Reserved for future Gazebo-world manipulation; no behavioural effect yet. |
| `dispense_start_service_name` | string | `dispense_start` | Relative open-valve service name. |
| `dispense_stop_service_name` | string | `dispense_stop` | Relative close-valve service name. |

`frame_id` is **not** a parameter — it arrives per goal. `tool_frame` is always
set by the executor to `{tool_sn}_calibrated_tcp`, which must equal the
dispenser's published calibrated TCP frame id.

## Lifecycle behaviour

- **configure**: read parameters, build the config DTO, create the movement
  action client, the end-tool lifecycle manager, and the movement dispatcher.
- **activate**: activate the end tool (already-active → warn and continue;
  `inactive`→`active` failure → activation fails; any other state → activation
  fails), verify the movement action server, create the dispenser service
  clients, create the latched `~/executing_patterns` publisher (publishing
  `false`), then create the action server. Goals are only accepted while active.
- **deactivate**: destroy the action server, abort any in-flight goal, destroy
  the dispenser clients, deactivate the end tool, publish `false`, and destroy
  the publisher.
- **cleanup**: destroy the movement client and end-tool manager, drop config.

`~/executing_patterns` (`std_msgs/Bool`, latched `transient_local`, depth 1) is
`true` while a goal executes and `false` otherwise. The publisher exists only
while the node is active.

## Build and run

```bash
colcon build --symlink-install --packages-select pattern_executors
source install/setup.bash

ros2 launch pattern_executors dispensing_pattern_executor.launch.py \
    tool_sn:=abc123 endtool_node_name:=dispenser_abc123
```

The node is a lifecycle node; drive it with `ros2 lifecycle set
<node> configure` / `activate` (the `movement_controller` and the target
dispenser node are assumed to be running already).

## Integration assumptions

- `movement_controller` is already active; this package is only an action
  client of it.
- The end tool is a lifecycle node exposing the standard lifecycle/parameter
  services plus `~/dispense_start` and `~/dispense_stop`, and is left
  `inactive` (or already `active`) before the executor is activated.
- The product/heatsink origin frame named by the goal `frame_id` is published
  by another node.

## Out of scope

Automated tests, assembly-JSON parsing, planning-scene/world injection, and
end-tool selection/mounting are handled elsewhere and are intentionally not part
of this package.
