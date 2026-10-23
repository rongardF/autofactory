# `pattern_executors` — Implementation Specification

Status: **Final**
Scope of this document: the new `pattern_executors` ROS 2 package and its first
concrete node, **`DispensingPatternExecutor`**, including the abstract
`PatternExecutor` base, the ROS 2 interfaces, the execution algorithm, and the
integration contracts with `movement_controller` and `endtools`.

> This spec is derived from [raw_ideas.md](raw_ideas.md) and
> [example_assembly.json](example_assembly.json), and from three rounds of
> clarification. Decisions that were explicitly confirmed are marked
> **[CONFIRMED]**. All items are resolved; see the decision log in
> [Section 12](#12-decision-log-confirmed).
>
> **Implementation note:** this `SPEC.md` and `raw_ideas.md` are planning
> artifacts that will **not** be committed and will not be available later. The
> implementation and code comments must therefore **never reference** these
> documents (no "see SPEC.md §x", no "per raw_ideas.md"). Any rationale that
> needs to live on must be written directly into code comments, docstrings, or
> the package `README.md`.

---

## 1. Purpose and context

The `pattern_executors` package executes manufacturing *patterns* on a robot
arm. A **pattern** is a sequence of robot movements, each optionally carrying
end-tool *actions* (e.g. open/close a dispenser valve, wait). Different
manufacturing operations (dispensing, screwdriving, component mounting, …) each
get their own executor node, but all share one abstract contract.

This milestone delivers:

1. The **abstract `PatternExecutor`** lifecycle node (shared interface,
   lifecycle, concurrency guard, end-tool lifecycle management, movement
   client, `executing_patterns` status topic).
2. The concrete **`DispensingPatternExecutor`** node (dispensing-specific
   actions, metrics, and flow-rate handling).

Out of scope (handled by other nodes, assumed already running):

- Parsing the assembly JSON into ROS messages — a separate orchestrator parses
  the assembly and calls this node's action with already-built messages.
  **[CONFIRMED]**
- Injecting the product/heatsink model into the MoveIt 2 planning scene (and,
  in simulation, into the Gazebo world).
- Mounting/selecting the physical end tool (the orchestrator does this using
  `operation_args` before activating this node).
- Publishing the product/heatsink origin frame used as `frame_id`.
- Generating shake-off movements — these are **already baked into** the incoming
  pattern movements; the executor runs them verbatim. **[CONFIRMED]**

---

## 2. Dependencies and integration contracts

### 2.1 `movement_controller` (action client)

- Action: `movement_controller/execute_trajectory`, type
  [`movement_controller/action/ExecuteTrajectory`](../movement_controller/action/ExecuteTrajectory.action).
- Goal: `movement_controller/TrajectoryPath[] paths`.
- Feedback: `string status` (`"executing"` → emitted when a blend-group starts;
  `"completed"` → emitted when that group finishes) and
  `string[] trajectory_path_ids` (the path IDs in that blend-group).
- Result: `bool success`, `string error_message`,
  `string[] trajectory_paths_completed`.
- **Assumed already active.** The executor is only an action client and never
  manages `movement_controller`'s lifecycle. It calls `wait_for_server()` with
  a timeout. **[CONFIRMED]**

Path message
[`movement_controller/msg/TrajectoryPath`](../movement_controller/msg/TrajectoryPath.msg)
fields reused directly: `path_id`, `motion_type` (`LIN`/`PTP`/`CIRC`),
`geometry_msgs/PoseStamped target_pose`, `blend_radius`, `cartesian_speed`,
`cartesian_acceleration`, `joint_speed`, `joint_acceleration`, `tool_frame`,
`circ_type` (`interim`/`center`), `geometry_msgs/Point circ_point`.

### 2.2 `endtools` — volumetric dispenser (lifecycle + services)

Target node:
[`volumetric_dispensing_tool`](../endtools/endtools/volumetric_dispensing_tool.py).

- `~/dispense_start` (`std_srvs/Trigger`) → logical **open_valve**.
- `~/dispense_stop` ([`endtools/StopDispensing`](../endtools/srv/StopDispensing.srv))
  → logical **close_valve**; returns `success`, `dispensed_volume`,
  `duration_seconds`, `error_message`.
- There is **no** literal `open_valve`/`close_valve` service; the executor maps
  those action names onto `dispense_start` / `dispense_stop`.
- `flowrate` (cc/s) is a node parameter read **only in `on_configure`**
  ([volumetric_dispensing_tool.py](../endtools/endtools/volumetric_dispensing_tool.py#L326)),
  so changing it requires a reconfigure cycle (see §7.3).
- Standard lifecycle services (`change_state`, `get_state`, `set_parameters`,
  `get_parameters`) are used to activate/deactivate/reconfigure the end tool.

### 2.3 End-tool lifecycle management

The executor drives the end-tool lifecycle **by node name** using the same
logic as `NodeStateManager`
([node_state_manager.py](../tools_manager/tools_manager/services/node_state_manager.py)).

> Per project direction, this logic is **re-implemented** inside
> `pattern_executors` (copied, not imported) to keep the packages decoupled at
> this stage. **[CONFIRMED]** The re-implementation must preserve the
> wall-clock (not node-clock) polling and the separate `ReentrantCallbackGroup`
> to avoid the documented sim-time / transition-callback deadlocks.

---

## 3. Package layout

The package follows the `tools_manager` package structure: node implementations
at the package root, and purpose-specific **singular-named** sub-packages
(`interface`, `enumerator`, `model`, `service`, `util`). The abstract base
lives in `interface/`.

```text
src/pattern_executors/
├── action/
│   └── ExecuteDispensingPattern.action
├── msg/
│   ├── DispensingPattern.msg
│   ├── DispensingMovement.msg
│   ├── PatternAction.msg
│   ├── DispensingPatternMetric.msg
│   └── DispensedElementMetric.msg
├── pattern_executors/
│   ├── __init__.py
│   ├── dispensing_pattern_executor.py      # concrete node (package root)
│   ├── interface/
│   │   ├── __init__.py
│   │   └── pattern_executor.py              # abstract PatternExecutor (LifecycleNode)
│   ├── enumerator/
│   │   ├── __init__.py
│   │   ├── pattern_action_type_enum.py      # OPEN_VALVE / CLOSE_VALVE / WAIT
│   │   └── action_timing_enum.py            # START / END
│   ├── model/
│   │   ├── __init__.py
│   │   ├── pattern_executor_config_dto.py
│   │   └── dispensing_executor_config_dto.py
│   ├── service/
│   │   ├── __init__.py
│   │   ├── endtool_lifecycle_manager.py     # re-implemented NodeStateManager
│   │   └── movement_dispatcher.py           # batching + action-timed execution
│   └── util/
│       ├── __init__.py
│       └── units_converter.py               # m³/s ↔ cc/s
├── config/
│   └── dispensing_pattern_executor.yaml
├── launch/
│   └── dispensing_pattern_executor.launch.py
├── scripts/
│   └── dispensing_pattern_executor           # installed console entry point
├── CMakeLists.txt         # ament_cmake_python + rosidl_generate_interfaces
├── package.xml
└── README.md
```

Package type: `ament_cmake` with `ament_cmake_python` and
`rosidl_generate_interfaces` (mixed Python + generated interfaces, per project
convention). `package.xml` must `<depend>` on `movement_controller` (for
`TrajectoryPath`), `std_msgs`, `std_srvs`, `geometry_msgs`, `lifecycle_msgs`,
and `rcl_interfaces`.

> Tests are **out of scope** for this iteration and are intentionally omitted
> from the layout. ([Section 11](#11-testing-out-of-scope) records this.)

License: BSD-3-Clause header on every source file.

---

## 4. ROS 2 interfaces

### 4.1 `PatternAction.msg`

```text
# Action type constants
string TYPE_OPEN_VALVE="open_valve"
string TYPE_CLOSE_VALVE="close_valve"
string TYPE_WAIT="wait"

# Timing constants
string TIMING_START="start"   # fire when the movement's trajectory starts (on 'executing' feedback)
string TIMING_END="end"       # fire when the movement's trajectory completes (on 'completed' feedback)

string type        # one of TYPE_*
string timing      # one of TIMING_*
float64 wait_time_s 0.0   # only used when type == TYPE_WAIT; blocking
```

Rationale for explicit `wait_time_s` rather than an arbitrary key/value map: the
only action argument in scope is the wait duration; an explicit typed field is
clearer and matches the project's "no arbitrary dicts across boundaries"
convention. Additional dispensing action args can be added as fields later.

### 4.2 `DispensingMovement.msg`

```text
movement_controller/TrajectoryPath path
PatternAction[] actions
```

The executor forwards `path` to `movement_controller` unchanged **except** for
`path.tool_frame`, which the **executor** always overwrites with
`f"{tool_sn}_calibrated_tcp"` from its `tool_sn` parameter (the orchestrator
does not set it). It fires `actions` based on feedback (§7.5). The orchestrator
is responsible for setting `path.path_id` and all motion fields, and guarantees
that any movement carrying a precise-timing action has **no blending** with
neighbours. **[CONFIRMED]**

### 4.3 `DispensingPattern.msg`

```text
string pattern_id
float64 flowrate_m3_s                 # per-pattern dispenser flow rate (see §7.3)
geometry_msgs/PoseStamped start_pose  # pattern origin; valve-closed PTP approach target (§7.2)
DispensingMovement[] movements
```

### 4.4 `ExecuteDispensingPattern.action`

> Each pattern-executor type defines its **own** action interface. This is the
> dispensing executor's action; future executors define their own (e.g.
> `ExecuteScrewdrivingPattern.action`) with their own pattern/metric types.

```text
# Goal
DispensingPattern[] patterns
string frame_id          # frame all PoseStamped poses are expressed in (product/heatsink origin)
string operation_id      # from assembly operation.id
---
# Result
bool success
string error_message
float64 execution_time_s
string operation_id
DispensingPatternMetric[] metrics     # one per executed pattern
---
# Feedback (emitted after each completed pattern)
string pattern_id
float64 execution_time_s              # wall-clock for that pattern
DispensingPatternMetric metric
```

> The action is **type-specific**: `DispensingPatternExecutor` uses this
> `ExecuteDispensingPattern` action carrying `DispensingPattern[]`. Other future
> executors define their own analogous action with their own pattern/metric
> types. The common contract (goal carries `frame_id` + `operation_id`,
> result/feedback shape, reject-concurrent, abortable) is shared by convention,
> enforced in the abstract base via generics/hooks (§6).

### 4.5 Metric messages

```text
# DispensingPatternMetric.msg
string pattern_id
float64 dispensed_volume_m3            # summed over all dispensed elements in the pattern
float64 pattern_execution_time_s      # wall-clock for the whole pattern
DispensedElementMetric[] element_metrics
```

```text
# DispensedElementMetric.msg
string element_id                      # path_id of the movement whose open_valve started this element
float64 dispensed_volume_m3
float64 dispensing_time_s
```

One `DispensedElementMetric` is produced per open→close cycle (one dispensed
dot or stroke). `element_id` is the `path_id` of the movement whose `open_valve`
started that element. **[CONFIRMED]** `dispensed_volume_m3` and
`dispensing_time_s` come from the
`dispense_stop` response (`dispensed_volume`, `duration_seconds`), converting
cc→m³ (×1e-6). The pattern-level `dispensed_volume_m3` and
`pattern_execution_time_s` are the summed/whole-pattern values. **[CONFIRMED]**

### 4.6 `executing_patterns` topic

- Name: `~/executing_patterns`
- Type: `std_msgs/Bool`
- QoS: `transient_local` (latched), depth 1, reliable.
- Publishes `true` while a goal is executing, `false` otherwise.
- The publisher is **created on activation** and **destroyed on deactivation**
  (see §7.1). A latched `false` is published immediately after the publisher is
  created. **[CONFIRMED]**

---

## 5. Parameters

Declared on the abstract base unless noted. All declared explicitly with
`ParameterDescriptor`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tool_sn` | string | `""` (required) | End-tool serial number. Movement frame is `{tool_sn}_calibrated_tcp`. **[CONFIRMED]** |
| `endtool_node_name` | string | `""` (required) | Node name of the end tool whose lifecycle this executor drives. |
| `movement_action_name` | string | `movement_controller/execute_trajectory` | Action name of the movement controller. |
| `movement_server_timeout_s` | double | `5.0` | `wait_for_server` timeout. |
| `endtool_service_timeout_s` | double | `5.0` | Lifecycle/param/service call timeout for the end tool. |
| `simulated` | bool | `true` | Declared for future Gazebo-world manipulation; **no behavioral effect yet**. **[CONFIRMED]** |

`frame_id` is **not** a parameter — it arrives per goal. Movement `tool_frame`
is always set to `f"{tool_sn}_calibrated_tcp"`; this must equal the dispenser's
published `calibrated_tcp_frame_id`. **[CONFIRMED]**

---

## 6. Abstract `PatternExecutor` design

`class PatternExecutor(LifecycleNode)` — shared, pattern-type-agnostic behavior.

Owns:
- The movement-controller action client.
- The `~/executing_patterns` latched publisher + concurrency state
  (`_is_executing` guarded by a lock).
- The end-tool `EndtoolLifecycleManager`.
- A type-specific action server provided by the concrete subclass (generic
  contract; see below).
- A `MultiThreadedExecutor` with `ReentrantCallbackGroup`s (movement feedback,
  end-tool clients, and action execution must run concurrently without
  deadlock).

The base class action-server contract is explicitly generic: the abstract base
manages concurrency/lifecycle/error-handling flow and owns a
`rclpy.action.ActionServer[GoalT, ResultT, FeedbackT]`, while each concrete
executor provides its own action type (e.g. `ExecuteDispensingPattern`) via a
subclass hook/factory.

> **Interface lifecycle:** the action server and the `~/executing_patterns`
> publisher are **created only on activation** and **destroyed on
> deactivation** (§7.1), so no goals can be accepted and no status is advertised
> while the node is merely configured/inactive. **[CONFIRMED]**

Abstract hooks implemented by subclasses:

| Hook | Responsibility |
|------|----------------|
| `_on_endtool_activated()` | Create type-specific end-tool clients (e.g. dispense start/stop) after the end tool is active. |
| `_on_endtool_deactivating()` | Destroy those clients. |
| `_prepare_pattern(pattern)` | Pre-pattern setup (e.g. flow-rate reconfigure). |
| `_fire_action(action, context)` | Execute one `PatternAction` (open/close/wait). |
| `_build_pattern_metric(...)` | Aggregate element metrics into a `DispensingPatternMetric`. |
| `_abort_cleanup()` | Type-specific safe-stop (e.g. force `dispense_stop`). |

Concurrency guard: the action server's goal-accept callback **rejects** a new
goal whenever `_is_executing` is true or the node is not active. **[CONFIRMED]**
Goal acceptance and transition to executing state must be atomic under a single
lock: once a goal is accepted, `_is_executing` is set within the same critical
section so no second goal can be accepted concurrently.

---

## 7. Execution algorithm (`DispensingPatternExecutor`)

### 7.1 Lifecycle

- **`on_configure`**: declare/read params; build config DTO
  (`DispensingExecutorConfigDTO`); create the movement action client; create the
  `EndtoolLifecycleManager` (clients to the end tool). **No** action server and
  **no** `~/executing_patterns` publisher are created here. Any
  `ValidationError` → `FAILURE`.
- **`on_activate`**: activate the end tool via `change_state`.
  - If the end tool is **already `active`** → log a warning and continue.
    **[CONFIRMED]**
  - If the end tool is `inactive`, request `inactive -> active`; if that
    transition fails, executor activation **FAILS**. **[CONFIRMED]**
  - If the end tool is not `inactive` and not `active` (for example
    `unconfigured`), executor activation **FAILS** because this executor does
    not perform pre-activation configure transitions. **[CONFIRMED]**
  - Verify the movement action server is reachable
    (`wait_for_server(timeout)`); call `_on_endtool_activated()` to create the
    dispense clients.
  - **Create** the `~/executing_patterns` publisher and publish `false`, then
    **create** the `ExecuteDispensingPattern` action server (so goals are only
    accepted while active). **[CONFIRMED]**
- **`on_deactivate`**: **destroy** the action server (stop accepting goals);
  if a goal is currently executing, first force the same abort sequence defined
  in §7.6 (cancel in-flight movement goal, run `_abort_cleanup()`, publish
  `executing_patterns=false`) before continuing lifecycle deactivation.
  `_on_endtool_deactivating()`; deactivate the end tool via `change_state`;
  publish `executing_patterns=false` and then **destroy** the publisher.
  **[CONFIRMED]**
- **`on_cleanup`**: destroy the movement action client and the
  `EndtoolLifecycleManager`; drop config.

### 7.2 Goal handling (overview)

Goal validation is performed before acceptance. A goal with `patterns` length
equal to 0 is invalid and must be rejected. A goal containing any invalid
`PatternAction` value is invalid and must be rejected.

Before each pattern's movements, the executor issues a **valve-closed `PTP`
approach to `pattern.start_pose`** (no actions attached). This positions the
tool at the pattern origin so the first stroke does not drag a putty trail from
an arbitrary current pose. The approach is the **executor's** responsibility and
is not present in `movements[]`. The approach pose is stamped with the goal
`frame_id` exactly like all movement poses (`target_pose.header.frame_id =
goal.frame_id`). **[CONFIRMED]**

```
accept goal (reject if busy/not active)
publish executing_patterns = true
t0 = now
metrics = []
try:
    for pattern in goal.patterns:
        _prepare_pattern(pattern)            # flow-rate reconfigure (§7.3)
        approach to pattern.start_pose       # PTP, valve closed, no actions (§7.2)
        pm = execute_pattern(pattern, goal.frame_id)   # §7.4–7.5
        metrics.append(pm)
        publish_feedback(pattern.pattern_id, pm.pattern_execution_time_s, pm)
    result.success = true
finally handling abort/cancel/error → §7.6
publish executing_patterns = false
result.execution_time_s = now - t0
result.operation_id = goal.operation_id
result.metrics = metrics
succeed()
```

### 7.3 Per-pattern flow-rate reconfigure

`flowrate_m3_s` is converted to cc/s (`× 1e6`) and compared with the end tool's
current `flowrate` parameter (read via `get_parameters`). Comparison is done to
6 fractional digits (round both values to 6 digits after the decimal point
before equality check). If it differs, the executor reconfigures the end tool:
**deactivate → cleanup → set `flowrate` param → configure → activate**
(because `flowrate` is only read in `on_configure`). If it matches, no
reconfigure. **[CONFIRMED]**

> Reconfiguring mid-goal re-runs the end tool's configure/activate; the executor
> must re-verify the end tool reaches `active` and fail the goal (with safe
> cleanup) if it does not.

### 7.4 Batching rule

A pattern's `movements[]` are split into **execution batches**. A batch boundary
is cut **immediately after any movement that carries a blocking `wait` action**.
Each batch is sent as **one** `ExecuteTrajectory` goal (blending preserved
within a batch). Between batches, no movement goal is in flight, so a blocking
wait holds the robot in place. Continuous strokes with no blocking wait flow as
a single blended goal. **[CONFIRMED]**

`wait` actions are only valid on zero-length movements. If a non-zero-length
movement contains a `wait` action, the goal is invalid and must be rejected.

### 7.5 Action-timed execution within a batch

For each `ExecuteTrajectory` goal (batch):

1. Build `TrajectoryPath[]` from the batch's `DispensingMovement.path` entries,
   stamping `target_pose.header.frame_id = goal.frame_id` and
   `tool_frame = {tool_sn}_calibrated_tcp`.
2. Send the goal; subscribe to feedback.
3. On feedback `status == "executing"` with `trajectory_path_ids = G`: for each
   movement whose `path.path_id ∈ G`, fire its `timing == "start"` actions **in
   listed order**. **[CONFIRMED]**
4. On feedback `status == "completed"` with `trajectory_path_ids = G`: for each
   such movement, fire its `timing == "end"` actions in listed order.
5. Start-action processing for a movement must fully complete (including a
   blocking `wait`) **before** its end actions run, even if the `"completed"`
   feedback has already arrived (per-movement serialization). This is what lets
   a dot (`open_valve(start)` → `wait(start)` → `close_valve(end)` on one
   zero-length movement) hold the valve open in place.
6. Await the goal result; on failure raise to the goal-level error handler.

Feedback handling is idempotent per movement `path_id`: start actions and end
actions may each execute at most once. Duplicate feedback for already-fired
start/end phases is ignored. If a `completed` feedback is observed before start
actions have completed for the same movement, end-action execution waits for the
start-action completion barrier before running.

Action semantics:
- `open_valve` → call `~/dispense_start` (`Trigger`). Non-blocking.
- `close_valve` → call `~/dispense_stop` (`StopDispensing`); record a
  `DispensedElementMetric` from the response. Non-blocking (service round-trip).
- `wait` → block the executor for `wait_time_s` (no movement dispatched during
  the wait). **[CONFIRMED]**

`open_valve`/`close_valve` are non-blocking with respect to robot motion
execution (movement goal keeps running while service calls are in flight), but
each action still has completion semantics for ordering: the dispatcher must
await a service response before considering that action finished and before
running the next action for the same movement.

Because precise-timing movements are guaranteed unblended, each forms its own
single-path blend-group, so `"executing"`/`"completed"` carry exactly that
`path_id` and timing is exact.

### 7.6 Abort / cancel / failure

The goal is **abortable**. **[CONFIRMED]** On cancel request, movement failure,
end-tool failure, or any exception mid-goal:

1. **Cancel the in-flight `movement_controller` goal** (async cancel + await).
   **[CONFIRMED]**
2. `_abort_cleanup()` → force `~/dispense_stop` to guarantee the valve is closed
   (dispensing must not leak). **[CONFIRMED]**
3. Publish `executing_patterns = false`; set `result.success = false` with
   `error_message`; `goal_handle.abort()` (or `canceled()` on explicit cancel).
4. Always log the error before returning the failure result (project
   convention).

Result metric policy on failure/cancel: return metrics only for patterns that
fully completed before the abort/cancel boundary; do not return partial metrics
for an in-progress pattern.

---

## 8. Internal data models (Pydantic v2)

Used for config/validation inside the package (not across ROS boundaries):

- `PatternExecutorConfigDTO` (frozen): `tool_sn`, `endtool_node_name`,
  `movement_action_name`, `movement_server_timeout_s`,
  `endtool_service_timeout_s`, `simulated`. Validates `tool_sn` /
  `endtool_node_name` non-empty. Exposes `calibrated_tcp_frame` property
  (`f"{tool_sn}_calibrated_tcp"`).
- `DispensingExecutorConfigDTO(PatternExecutorConfigDTO)`: dispense service
  names.

Enums (str, Enum; `*Enum` suffix): `PatternActionTypeEnum`
(`OPEN_VALVE`/`CLOSE_VALVE`/`WAIT`), `ActionTimingEnum` (`START`/`END`). All DTO
fields carry descriptions; DTOs are frozen where not mutated.

---

## 9. Threading & callback groups

- `MultiThreadedExecutor` ownership follows `tools_manager`: the node's
  executable entry point (`main`) creates the executor, adds the node, and
  spins it.
- Action execution runs in its own reentrant group.
- End-tool lifecycle/service clients live in a separate reentrant group (as in
  `NodeStateManager`), so synchronous waits inside transition callbacks don't
  deadlock.
- Movement feedback callbacks run in a reentrant group so start/end actions can
  fire while the action-execution coroutine awaits the movement result.
- Lifecycle/service waits poll on the **wall clock** (not node clock) to stay
  correct under `use_sim_time`.
- Execution-time metrics (`execution_time_s`, `pattern_execution_time_s`) are
  measured on the **node clock** so they report simulation time under
  `use_sim_time`, consistent with the end tool's `dispensing_time_s`.
- `wait` pattern actions sleep on the **node clock** (`Clock.sleep_for`), so
  they last the requested simulation time; a paused simulation pauses the wait.

---

## 10. Error handling conventions

- At the action boundary, always return an `ExecuteDispensingPattern.Result` with
  `success` + `error_message`; never let exceptions propagate out of the action.
- Internal failures may raise; they are caught at the goal handler and converted
  to a failure result after `_abort_cleanup()`.
- Log before returning any failure.
- Input validation failures (for example empty goal patterns array, invalid
  `PatternAction.type` or `PatternAction.timing`, invalid wait placement) are
  treated as invalid goals and rejected at goal-accept time.

---

## 11. Testing (out of scope)

Tests are **intentionally not implemented in this iteration**. No `tests/`
directory, no `ament_pytest`/`pytest` test dependencies, and no test targets are
added now. When tests are introduced later, the mocking strategy (fake
`movement_controller` action server emitting `executing`/`completed` feedback;
fake end-tool lifecycle + `dispense_start`/`dispense_stop` services) and the
coverage areas (batching, action ordering, start-before-end serialization,
flow-rate conversion/reconfigure, metric aggregation, concurrency rejection,
abort path, end-tool activation rules) should be revisited.

---

## 12. Decision log (confirmed)

1. Executor consumes parsed `DispensingPattern[]` messages; never parses JSON.
2. Shake-off movements are baked into incoming movements; run verbatim.
3. Movements sent as `TrajectoryPath[]`; blending preserved; precise-timing
   movements are guaranteed unblended by the orchestrator; `executing`→start,
   `completed`→end; no empty-movement trick.
4. Reuse `movement_controller/TrajectoryPath`; poses Euler `[x,y,z,rx,ry,rz]`
   at the message layer become `PoseStamped`; `circ_point` is a point.
5. `operation_id` comes in the goal; `operation_args`/`pattern_args` are not in
   the goal (orchestrator uses them upstream), except `flowrate_m3_s` which is a
   per-pattern field in `DispensingPattern`.
6. `executing_patterns` = `std_msgs/Bool`, latched (`transient_local`, depth 1).
7. End-tool lifecycle driven via `change_state` (NodeStateManager logic
  re-implemented, not imported); deactivates on `on_deactivate`; already-active
  → warn + continue; inactive→active transition failure → activation fails;
  non-inactive/non-active states (e.g. unconfigured) fail activation.
8. `movement_controller` assumed active; executor is only a client.
9. Movement frame always `{tool_sn}_calibrated_tcp`; `tool_sn` is a param.
10. `simulated` declared but no behavior yet.
11. Abstraction now: abstract `PatternExecutor` + concrete
    `DispensingPatternExecutor`.
12. Flow-rate converted m³/s→cc/s and set on the end tool per pattern (with a
    reconfigure cycle) only when it differs.
13. On abort: cancel in-flight movement goal **and** force `dispense_stop`;
    action is abortable.
14. Actions execute in listed order; `wait` blocks the executor.
15. `close_valve` is on the same movement as its `open_valve`/`wait`.
16. `DispensedElementMetric` is keyed by `element_id` = the `path_id` of the
    movement whose `open_valve` started that element.
17. The executor performs a valve-closed `PTP` approach to `pattern.start_pose`
    (no actions) before each pattern's movements.
18. The **executor** (not the orchestrator) always sets each `path.tool_frame`
    to `{tool_sn}_calibrated_tcp` from its `tool_sn` parameter.
19. The dispensing action interface is named `ExecuteDispensingPattern.action`;
    each pattern-executor type defines its own action.
20. The action server and `~/executing_patterns` publisher are created on
    `on_activate` and destroyed on `on_deactivate`.
21. Package structure follows `tools_manager`: node files at the package root;
  singular sub-packages (`interface`, `enumerator`, `model`, `service`,
  `util`); the abstract base lives in `interface/`.
22. Tests are out of scope for this iteration.
23. Planning docs (`SPEC.md`, `raw_ideas.md`) are not committed and must never
    be referenced from code or comments.
24. `PatternExecutor` keeps a generic action-server contract; concrete
  executors provide their own action type.
25. On failure/cancel, result metrics include only fully completed patterns;
  partial in-progress pattern metrics are omitted.
26. Goal acceptance and `_is_executing` transition are atomic under a single
  lock; accepted goal implies busy state is already set.
27. A goal with empty `patterns[]` is invalid and rejected.
28. Any invalid `PatternAction` value is invalid input and causes goal
  rejection.
29. `wait` actions are valid only on zero-length movements; otherwise the goal
  is rejected as invalid.
30. Feedback processing is idempotent per `path_id`; duplicate start/end
  feedback is ignored and end actions are serialized after start completion.
