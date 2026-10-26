# `endtools`

ROS 2 (Jazzy) package that provides the per-tool nodes that drive a robot
end-effector ("endtool"). Today it ships a single endtool — a **volumetric
dispenser** — plus the Gazebo model and detachable-joint wiring needed to
simulate that tool being carried on a tool rack and a robot tool-mount.

This document describes the behaviour **as implemented in the code**, so it can
be used to verify that the package does what is intended.

---

## What it contains

| Artifact | Type | Purpose |
|----------|------|---------|
| `volumetric_dispensing_tool` | Lifecycle node (executable) | Controls one dispensing tool, exposes dispense start/stop and set_tcp services, broadcasts the calibrated TCP frame, and publishes the calibrated TCP's expiry time. |
| `srv/StopDispensing.srv` | Service | Stop request/response carrying dispensed volume + duration. |
| `srv/SetTcp.srv` | Service | Set the calibrated TCP at runtime; persists it to `station_cache`. |
| `launch/volumetric_dispenser_launch.py` | Launch file | Starts the node and (in sim) the Gazebo↔ROS bridge for its detachable joints. |
| `model/dispensing_tool/` | Gazebo model | XACRO→SDF model with two detachable joints and an RFID tag plugin. |

### Python layout (`endtools/endtools/`)

- `volumetric_dispensing_tool.py` — the node (see below).
- `interface/dispenser_controller.py` — `DispenserController` ABC.
- `service/simulated_dispenser_controller.py` — mock dispenser used in sim.
- `service/hardware_dispenser_controller.py` — real-hardware dispenser (**stubbed**, see Observations).
- `model/*` — Pydantic v2 DTOs (`DispensingToolConfigDTO`, `DispensingMetricsDTO`, `ToolConfigBaseDTO`, `ToolMetadataInfoDTO`).
- `enumerator/tool_type_enum.py` — `ToolTypeEnum` (`UNKNOWN`, `VOL_DISPENSER`).
- `utils/transformations.py` — pure (node-free) TCP transform math.

---

## The node: `VolumetricDispensingTool`

A `rclpy` **LifecycleNode**. All runtime wiring is bound to lifecycle
transitions; nothing is created in `__init__` except parameter declarations.

### Parameters

| Name | Default | Meaning |
|------|---------|---------|
| `tool_type` | `"dispensing"` | Read-only tool type tag. Parsed into `ToolTypeEnum`. |
| `simulated` | `True` | Selects `SimulatedDispenserController` vs `HardwareDispenserController`. |
| `tool_sn` | `"dispensing_ABC123"` | Tool serial number; also the base of the calibrated TCP frame name and the TCP cache key. |
| `tcp_frame_id` | `"tool0"` | Parent frame the TCP offset is applied to (the flange). |
| `tcp_uncalibrated` | `[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797]` | Nominal (uncalibrated) tip pose `[x, y, z, roll, pitch, yaw]` (m, rad). Used as the fallback when no cached TCP is available. |
| `station_cache_name` | `"station_cache"` | Node name of the `station_cache` node providing the TCP store/fetch services. |
| `tcp_valid_period` | `86400.0` | Default validity period (seconds) for a cached TCP value. Clipped to a hard maximum of 3 days (see below). |
| `mounted` | `False` | Whether the tool is currently on the tool-mount. **Gates activation.** |
| `flowrate` | `1.0` | Volumetric flow rate (cc/s) used by the mock to integrate volume. |

### Lifecycle behaviour

**`on_configure`**
1. Reads all parameters into a frozen `DispensingToolConfigDTO` (Pydantic
   validates: `tcp_uncalibrated` must have 6 elements, `flow_rate >= 0`).
2. Creates a **latched** (`TRANSIENT_LOCAL`, depth 1) publisher `~/mounted` and
   immediately publishes the mounted state as a lowercased string
   (`"true"`/`"false"`).
3. Creates the `station_cache` service clients (`store_number_array` /
   `fetch_number_array`) used by the `tool_tcp` property to persist and
   retrieve the calibrated TCP under the key `dispenser_<tool_sn>_tcp`.
4. Publishes the calibrated and uncalibrated TCP frames as **static**
   transforms via a single **latched** (`TRANSIENT_LOCAL`, depth 1,
   `KEEP_LAST`) `/tf_static` publisher held for the node lifetime (the
   calibrated frame uses the current `tool_tcp`: the cached value or the
   uncalibrated fallback). The node keeps its own map of transforms keyed by
   `child_frame_id` and re-publishes the full set on every update, so a later
   `set_tcp` **replaces** the previously latched value for a frame instead of
   being dropped as a duplicate.
5. Creates the `~/set_tcp` service (`endtools/SetTcp`), which is available in
   **both** the inactive (configured) and active states.
6. Creates a **latched** (`TRANSIENT_LOCAL`, depth 1, `KEEP_LAST`)
   `~/tcp_expiration` publisher (`builtin_interfaces/Time`) and publishes the
   absolute expiry instant of the currently used calibrated TCP. The expiry is
   taken from the live `station_cache` entry backing the TCP; when the tool is
   falling back to the uncalibrated TCP (no live cached entry) the **current
   time** is published so the TCP is treated as already expired. Like
   `~/set_tcp`, this publisher is available in both the configured and active
   states (created on configure, destroyed on cleanup).
7. Instantiates the controller — `SimulatedDispenserController` when
   `simulated`, otherwise `HardwareDispenserController`.
8. Any `ValidationError` (or other exception) → `FAILURE`.

**`on_activate`** (refuses to activate unless the tool is actually mounted)
1. Fails if config is missing, **`mounted` is `False`**, or the controller is `None`.
2. `controller.setup(config)`.
3. Creates services:
   - `~/dispense_start` — `endtools/StartDispensing`
   - `~/dispense_stop` — `endtools/StopDispensing`

**`on_deactivate`** destroys the two dispense services and calls
`controller.teardown()`.

**`on_cleanup`** clears config, destroys the `~/set_tcp` service, the `~/mounted`
publisher, the latched `/tf_static` publisher (and clears the cached transform
map), the latched `~/tcp_expiration` publisher, and the `station_cache` clients,
resets the cached TCP and its expiration, and drops the controller.

The node is spun with a `MultiThreadedExecutor(num_threads=5)` and uses
`ReentrantCallbackGroup`s so service callbacks can run concurrently.

### Dispense start / stop flow

- **`~/dispense_start`** (`StartDispensing`): rejects if the controller is missing
  or already dispensing; otherwise calls `start_dispensing(pose_stamp)` and
  returns `success=True`. `RuntimeError`s are converted to `success=False` with
  an `error_message`.
- **`~/dispense_stop`** (`StopDispensing`): calls `stop_dispensing(pose_stamp)`,
  returning `success`, `dispensed_volume`, `duration_seconds`, and
  `error_message`.

Both requests carry a `pose_stamp`: the instant the TCP reached the pose that
starts/ends the dispensed element, supplied by the caller from the movement
feedback's `motion_stamp`. It is deliberately *not* the instant the service runs
— those differ by the caller's round-trip latency, and for a dwell they differ by
the whole dwell. A zero stamp means "sample the node clock" and is used by abort
cleanup, which has no motion to align to.

#### Simulated controller (`SimulatedDispenserController`)
- `start_dispensing(pose_stamp)` — sets the `is_dispensing` flag (thread-safe via
  `RLock`), records the valve-open time from the node clock and remembers the
  supplied pose stamp.
- `stop_dispensing(pose_stamp)` — clears the flag, computes
  `dispensed_volume = flow_rate × elapsed_seconds` from the **node clock** window
  (how long the valve was actually open) and returns a
  `DispensingMetricsDTO(dispensed_volume_cc, dispensing_duration_s, flowrate_cc)`.
- The deposited-material model is placed from the **pose stamps**: the TCP is
  looked up in TF at the start and end stamps, and the straight-line distance
  between them decides stripe vs. dot (`_STRIPE_MIN_LENGTH_MM`, above zero so a
  sub-millimetre residual of a nominally stationary dwell stays a dot).
- There is no physical actuation — the volume is pure time integration.

#### Hardware controller (`HardwareDispenserController`)
- `setup`/`teardown` store/clear config only.
- `start_dispensing()` and `stop_dispensing()` **raise `NotImplementedError`**
  (hardware I/O is a TODO).

### Setting the TCP (`~/set_tcp`)

- **`~/set_tcp`** (`endtools/SetTcp`): available in both the **configured** and
  **active** states (created on configure, destroyed on cleanup). The request
  carries `tcp` (`[x, y, z, roll, pitch, yaw]`, must have 6 elements) and an
  optional `tcp_valid_period` (seconds; `<= 0` means use the node default). The
  resolved validity period (request override or node default) is **clipped to a
  hard maximum of 3 days** — the dispensing tool TCP must always expire, so any
  larger value is clamped and a warning is logged. The value is stored in
  `station_cache` under `dispenser_<tool_sn>_tcp` **first**; only on a successful
  store is the cached value updated, the static TF re-published, and the latched
  `~/tcp_expiration` re-published with the new expiry. A failed store leaves the
  TCP, TF, and expiration unchanged, logs a warning, and returns `success=False`.
- Reading `tool_tcp` fetches from `station_cache` when unset; if the entry is
  missing or expired it falls back to `tcp_uncalibrated` and logs a warning.

### Publishing the TCP expiry (`~/tcp_expiration`)

- **`~/tcp_expiration`** (`builtin_interfaces/Time`, latched): broadcasts the
  absolute instant at which the currently used calibrated TCP expires. Published
  on configure and re-published on every successful `~/set_tcp`.
- The expiry comes from the live `station_cache` entry (`valid_till`) backing
  the TCP. If there is no live calibrated TCP (the tool is using the
  uncalibrated fallback), the **publish time** is sent so consumers treat the
  TCP as already expired.
- Because the TCP validity period is always clipped to at most 3 days, the
  published expiry is never further than 3 days into the future.

### Calibrated TCP math (`utils/transformations.py`)

Pure functions (no ROS node needed, so unit-testable):
- `rpy_to_quaternion` — intrinsic RPY (ZYX composition) → quaternion `(x,y,z,w)`.
- `build_tcp_transform(parent_frame_id, child_frame_id, offset, stamp)` — builds a
  fixed `parent_frame_id → child_frame_id` transform from a
  `[x, y, z, roll, pitch, yaw]` offset. The node calls this twice (once for the
  calibrated TCP, once for the uncalibrated TCP) and publishes both as **static**
  transforms relative to `tcp_frame_id` from `on_configure` (i.e. they are
  available as soon as the node reaches the *inactive* state).

---

## The Gazebo model (`model/dispensing_tool/model.sdf.xacro`)

A XACRO macro expanded to SDF at spawn time (by `tools_manager`, not by this
package). Key points:

- One visual (`visual.dae`, red material) and one collision (`collision.stl`).
- A `rfid_tag_link` carrying an `autofactory::rfid::RfidTag` plugin whose
  `<data>` is the hex-encoded tool tag payload (`tag_data` arg).
- **Two `gz-sim-detachable-joint-system` plugins** — one welding the tool to the
  **tool-mount** link, one to the **tool-rack slot** link. Each exposes
  `/<link>/<model>/{attach,detach,state}` Gazebo topics.
- Per the inline comment, in gz-sim 8.11.0 these joints **start attached**; the
  consumer must detach the redundant one after spawn (this is exactly what
  `tools_manager`'s Gazebo world manager does).

The macro parameters (`model_name`, `station_model_name`, `tool_mount_link`,
`tool_rack_link`, `tag_data`, …) are supplied by the spawner.

---

## Launch (`volumetric_dispenser_launch.py`)

Launch arguments: `simulated`, `tool_sn`, `tcp_frame_id`, `tcp_uncalibrated`,
`station_cache_name`, `tcp_valid_period`, `mounted`, `flowrate`,
`tool_rack_link` (required), `tool_mount_link` (default `tool_mount_tcp`).

It starts:
1. The `volumetric_dispensing_tool` node with the parameters above
   (`use_sim_time` is tied to `simulated`).
2. **Only when `simulated:=true`** — a `ros_gz_bridge parameter_bridge` that
   bridges the attach/detach/state topics for both the tool-mount link and the
   tool-rack link into ROS, so the detachable joints can be driven from ROS.

---

## Build / run

```bash
colcon build --symlink-install --packages-select endtools
source install/setup.bash

# Standalone (simulation), as a mounted tool:
ros2 launch endtools volumetric_dispenser_launch.py \
  tool_sn:=abc123 mounted:=true tool_rack_link:=slot1_attached_link

# Drive the lifecycle, then dispense:
ros2 lifecycle set /volumetric_dispensing_tool configure
ros2 lifecycle set /volumetric_dispensing_tool activate
ros2 service call /volumetric_dispensing_tool/dispense_start endtools/srv/StartDispensing
ros2 service call /volumetric_dispensing_tool/dispense_stop endtools/srv/StopDispensing
```

Interfaces are generated with `rosidl` from CMake (`ament_cmake` +
`ament_cmake_python`); the Python package is installed with
`ament_python_install_package`, and `gz_register_models(model)` puts the
`dispensing_tool` model on `GZ_SIM_RESOURCE_PATH`.

---

## Observations (for correctness verification)

These are discrepancies found while reading the code that affect whether the
package behaves as intended:

1. **Hardware path is a stub.** `HardwareDispenserController.start/stop` raise
   `NotImplementedError`; only the simulated path is functional.
