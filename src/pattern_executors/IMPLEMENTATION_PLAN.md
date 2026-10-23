# pattern_executors Implementation Plan

Source: `SPEC.md` in this same directory.
Date: 2026-10-21

## Goal
Implement the new ROS 2 package `pattern_executors` with:

- Abstract `PatternExecutor` lifecycle node.
- Concrete `DispensingPatternExecutor` node.
- ROS interfaces (messages + action).
- Movement dispatching, end-tool lifecycle management, abort behavior, and metrics.

## Scope Summary
In scope:

- New package scaffold under `src/pattern_executors/`.
- Interface definitions from the spec.
- Runtime nodes/services/utils/models/enums.
- Launch + config + entrypoint + README docs.
- Manual integration smoke validation.

Out of scope for this iteration:

- Automated tests.
- JSON parsing of assembly files.
- Planning scene/world injection.
- End-tool selection/mounting orchestration.

## Phase 1: Package Foundation

### Tasks
1. Create package layout exactly as specified.
2. Add build metadata:
   - `CMakeLists.txt` using `ament_cmake`, `ament_cmake_python`, `rosidl_generate_interfaces`.
   - `package.xml` dependencies for `movement_controller`, `std_msgs`, `std_srvs`, `geometry_msgs`, `lifecycle_msgs`, `rcl_interfaces`.
   - `setup.py` and `setup.cfg` for Python package install + console scripts.
3. Add Python package directories and `__init__.py` files:
   - `interface/`, `enumerator/`, `model/`, `service/`, `util/`.
4. Add BSD-3-Clause headers to all created source files.

### Deliverables
- Buildable package skeleton with generated Python package install path.

### Verification
- `colcon build --symlink-install --packages-select pattern_executors` succeeds.

## Phase 2: ROS Interface Definitions

### Tasks
1. Create `msg/PatternAction.msg`.
2. Create `msg/DispensingMovement.msg`.
3. Create `msg/DispensingPattern.msg`.
4. Create `msg/DispensedElementMetric.msg`.
5. Create `msg/DispensingPatternMetric.msg`.
6. Create `action/ExecuteDispensingPattern.action`.
7. Wire interface generation into CMake/package manifests.

### Deliverables
- Fully generated and importable interfaces exactly matching the spec.

### Verification
- Build succeeds.
- `ros2 interface show` works for all new message/action types.

## Phase 3: Internal Types and Utilities

### Tasks
1. Implement enums:
   - `PatternActionTypeEnum`: `OPEN_VALVE`, `CLOSE_VALVE`, `WAIT`.
   - `ActionTimingEnum`: `START`, `END`.
2. Implement Pydantic DTOs:
   - `PatternExecutorConfigDTO` (frozen, validated).
   - `DispensingExecutorConfigDTO`.
3. Implement `units_converter.py`:
   - `m3_s_to_cc_s`.
   - `cc_to_m3`.
4. Add field descriptions and immutability conventions per project rules.

### Deliverables
- Typed domain layer for config validation and unit conversions.

### Verification
- Node configure path can parse and validate parameters.

## Phase 4: End-tool Lifecycle Manager

### Tasks
1. Implement `service/endtool_lifecycle_manager.py` by re-implementing NodeStateManager logic inside this package.
2. Support lifecycle state transitions:
   - `get_state`, `change_state`, `configure`, `activate`, `deactivate`, `cleanup`.
3. Support parameter APIs:
   - `get_parameters`, `set_parameters`.
4. Use separate `ReentrantCallbackGroup` and wall-clock polling.
5. Respect timeout configuration and return explicit failure errors.

### Deliverables
- Decoupled lifecycle/parameter service layer for the target end-tool node.

### Verification
- Manual call path confirms activate/deactivate/reconfigure behavior including timeout and error handling.

## Phase 5: Movement Dispatcher

### Tasks
1. Implement `service/movement_dispatcher.py`.
2. Split movement sequence into batches using blocking `wait` boundary rule.
3. Send one `ExecuteTrajectory` goal per batch.
4. Stamp every movement path with:
   - `target_pose.header.frame_id = goal.frame_id`.
   - `tool_frame = f"{tool_sn}_calibrated_tcp"`.
5. Process action-timed feedback:
   - Start actions on `executing`.
   - End actions on `completed`.
6. Ensure idempotent firing per `path_id` and phase.
7. Enforce start-before-end serialization for each movement.

### Deliverables
- Dispatcher that preserves blending while enforcing action ordering constraints.

### Verification
- Simulated feedback sequence validates deduplication, ordering, and blocking wait behavior.

## Phase 6: Abstract PatternExecutor

### Tasks
1. Implement `interface/pattern_executor.py` as `LifecycleNode`.
2. Implement lifecycle methods:
   - `on_configure`: read params, build DTO, create movement client + lifecycle manager.
   - `on_activate`: activate end tool, verify movement server, create publisher and action server.
   - `on_deactivate`: destroy action server, abort if needed, deactivate end tool, publish false, destroy publisher.
   - `on_cleanup`: destroy remaining clients/managers and clear config.
3. Implement goal acceptance rules:
   - Reject when inactive.
   - Reject when currently executing.
   - Atomic lock around accept/busy transition.
4. Implement shared abort/cancel/failure flow:
   - Cancel in-flight movement goal.
   - Call subclass `_abort_cleanup()`.
   - Publish `executing_patterns=false`.
   - Return result object with explicit error info.
5. Define subclass hooks for type-specific behavior.

### Deliverables
- Reusable abstract execution framework for pattern-type executors.

### Verification
- Lifecycle transitions and concurrency rejection behave as specified.

## Phase 7: DispensingPatternExecutor

### Tasks
1. Implement `dispensing_pattern_executor.py` concrete node.
2. Implement hook methods:
   - `_on_endtool_activated`.
   - `_on_endtool_deactivating`.
   - `_prepare_pattern`.
   - `_fire_action`.
   - `_build_pattern_metric`.
   - `_abort_cleanup`.
3. Validate goal content at accept stage:
   - Non-empty patterns.
   - Valid action type/timing values.
   - `wait` only on zero-length movements.
4. Implement per-pattern flow:
   - Reconfigure flowrate if needed (round to 6 decimals in cc/s compare).
   - Run valve-closed PTP approach to `start_pose`.
   - Execute movement batches and timed actions.
   - Publish feedback after each completed pattern.
5. Implement metric capture:
   - Record one element metric per open-to-close cycle.
   - Convert dispenser volume cc to m3.
   - Return only completed pattern metrics on failure/cancel.

### Deliverables
- Functional dispensing executor aligned with all confirmed decision log items.

### Verification
- End-to-end action execution with normal completion, failure, and cancel paths.

## Phase 8: Runtime Wiring and Documentation

### Tasks
1. Create `config/dispensing_pattern_executor.yaml` defaults.
2. Create `launch/dispensing_pattern_executor.launch.py`.
3. Create `scripts/dispensing_pattern_executor` console entrypoint.
4. Ensure `main` uses `MultiThreadedExecutor` and adds/spins node.
5. Write package `README.md` with:
   - Action contract.
   - Parameters.
   - Lifecycle behavior.
   - Integration assumptions.
   - Known out-of-scope notes.

### Deliverables
- Launchable package with operator-facing documentation.

### Verification
- Node launches and can be activated/deactivated while exposing expected interfaces.

## Phase 9: Manual Integration Validation

### Tasks
1. Build and source workspace.
2. Bring up dependent nodes (`movement_controller` and target volumetric end-tool).
3. Run smoke scenarios:
   - Activation where end tool is already active.
   - One valid pattern with open/wait/close.
   - Cancellation during in-flight movement.
   - Mid-goal flowrate change causing reconfigure cycle.
4. Confirm `~/executing_patterns` latched behavior (`false` on activate and after all exits).
5. Inspect result payloads and logs for error handling conventions.

### Deliverables
- Validation notes proving core behavior against spec.

### Verification
- All critical spec behaviors from lifecycle, batching, action timing, and abort cleanup are observed.

## Task Breakdown by File Group

1. Build and package files:
   - `CMakeLists.txt`, `package.xml`, `setup.py`, `setup.cfg`.
2. ROS interfaces:
   - `action/ExecuteDispensingPattern.action`.
   - `msg/*.msg` files.
3. Core Python implementation:
   - `pattern_executors/interface/pattern_executor.py`.
   - `pattern_executors/dispensing_pattern_executor.py`.
   - `pattern_executors/service/endtool_lifecycle_manager.py`.
   - `pattern_executors/service/movement_dispatcher.py`.
   - `pattern_executors/model/*.py`.
   - `pattern_executors/enumerator/*.py`.
   - `pattern_executors/util/units_converter.py`.
4. Runtime assets:
   - `config/dispensing_pattern_executor.yaml`.
   - `launch/dispensing_pattern_executor.launch.py`.
   - `scripts/dispensing_pattern_executor`.
   - `README.md`.

## Risks and Mitigations

1. Callback deadlocks in lifecycle/service waits.
   - Mitigation: separate reentrant callback groups, wall-clock polling, timeout-driven failure paths.
2. Feedback race conditions causing duplicated or misordered actions.
   - Mitigation: per-`path_id` state tracking, idempotence checks, start-completion barrier before end.
3. Incorrect flowrate comparisons creating unnecessary reconfigure loops.
   - Mitigation: compare rounded cc/s values to 6 fractional digits exactly as specified.
4. Valve leak on failures.
   - Mitigation: mandatory abort cleanup calls `dispense_stop` after movement goal cancel.

## Acceptance Criteria

1. Package builds and generated interfaces resolve.
2. Executor only accepts goals while active and not busy.
3. `~/executing_patterns` is latched, accurate, and lifecycle-scoped.
4. Pattern execution flow includes pre-pattern approach move, batching, and timed actions.
5. Abort/cancel always cancels movement and closes valve.
6. Metrics are emitted only for fully completed patterns.
7. Implementation code and comments do not reference planning artifacts.

## Suggested Execution Order

1. Foundation.
2. Interfaces.
3. Types/utilities.
4. End-tool manager.
5. Movement dispatcher.
6. Abstract base.
7. Concrete dispensing executor.
8. Launch/config/docs.
9. Manual integration validation.
