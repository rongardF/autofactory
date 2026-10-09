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
| `volumetric_dispensing_tool` | Lifecycle node (executable) | Controls one dispensing tool, exposes dispense start/stop and set_tcp services, broadcasts the calibrated TCP frame. |
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
| `tcp_valid_period` | `86400.0` | Default validity period (seconds) for a cached TCP value. |
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
4. Publishes the calibrated and uncalibrated TCP frames as **static** transforms
   (the calibrated frame uses the current `tool_tcp`: the cached value or the
   uncalibrated fallback).
5. Creates the `~/set_tcp` service (`endtools/SetTcp`), which is available in
   **both** the inactive (configured) and active states.
6. Instantiates the controller — `SimulatedDispenserController` when
   `simulated`, otherwise `HardwareDispenserController`.
7. Any `ValidationError` (or other exception) → `FAILURE`.

**`on_activate`** (refuses to activate unless the tool is actually mounted)
1. Fails if config is missing, **`mounted` is `False`**, or the controller is `None`.
2. `controller.setup(config)`.
3. Creates services:
   - `~/dispense_start` — `std_srvs/Trigger`
   - `~/dispense_stop` — `endtools/StopDispensing`
4. Creates a `tf2_ros.TransformBroadcaster` and subscribes to `/tf` (depth 100).
   Whenever a `/tf` transform arrives whose `child_frame_id == tcp_frame_id`, it
   composes that transform with the static `tcp` offset and broadcasts the
   calibrated TCP frame `"<tool_sn>_calibrated_tcp"` back onto `/tf`.

**`on_deactivate`** destroys the two services, the `/tf` subscription and the
broadcaster publisher, and calls `controller.teardown()`.

**`on_cleanup`** clears config, destroys the `~/set_tcp` service, the `~/mounted`
publisher, the static TF broadcaster and the `station_cache` clients, resets the
cached TCP, and drops the controller.

The node is spun with a `MultiThreadedExecutor(num_threads=5)` and uses
`ReentrantCallbackGroup`s so service and `/tf` callbacks can run concurrently.

### Dispense start / stop flow

- **`~/dispense_start`** (`Trigger`): rejects if the controller is missing or
  already dispensing; otherwise calls `start_dispensing()` and returns
  `success=True`. `RuntimeError`s are converted to `success=False` with a message.
- **`~/dispense_stop`** (`StopDispensing`): calls `stop_dispensing()`, returning
  `success`, `dispensed_volume`, `duration_seconds`, and `error_message`.

#### Simulated controller (`SimulatedDispenserController`)
- `start_dispensing()` — sets the `is_dispensing` flag (thread-safe via `RLock`)
  and records the start time from the node clock.
- `stop_dispensing()` — clears the flag, computes
  `dispensed_volume = flow_rate × elapsed_seconds` and returns a
  `DispensingMetricsDTO(dispensed_volume_cc, dispensing_duration_s, flowrate_cc)`.
- There is no physical actuation — it is pure time integration.

#### Hardware controller (`HardwareDispenserController`)
- `setup`/`teardown` store/clear config only.
- `start_dispensing()` and `stop_dispensing()` **raise `NotImplementedError`**
  (hardware I/O is a TODO).

### Setting the TCP (`~/set_tcp`)

- **`~/set_tcp`** (`endtools/SetTcp`): available in both the **configured** and
  **active** states (created on configure, destroyed on cleanup). The request
  carries `tcp` (`[x, y, z, roll, pitch, yaw]`, must have 6 elements) and an
  optional `tcp_valid_period` (seconds; `<= 0` means use the node default).
  Assigning the `tool_tcp` property stores the value in `station_cache` under
  `dispenser_<tool_sn>_tcp` **first**; only on a successful store is the cached
  value updated and the static TF re-published. A failed store leaves the TCP
  and TF unchanged, logs a warning, and returns `success=False`.
- Reading `tool_tcp` fetches from `station_cache` when unset; if the entry is
  missing or expired it falls back to `tcp_uncalibrated` and logs a warning.

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
ros2 service call /volumetric_dispensing_tool/dispense_start std_srvs/srv/Trigger
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
