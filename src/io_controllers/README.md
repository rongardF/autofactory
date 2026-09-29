# `io_controllers`

Device-agnostic GPIO controller for ROS 2 Jazzy. A single `gpio_controller`
lifecycle node exposes a **normalized** I/O interface (read via topic, write via
service, watch via action) and delegates all hardware specifics to a pluggable
driver, selected from the config file (or the built-in mock when running
simulated). Adding new hardware means writing one new driver subclass — the
public interface never changes. Logical `io_name`s decouple application code
from raw pin numbers.

The first concrete driver targets Universal Robots (`ur`), wrapping
`ur_robot_driver`'s `io_and_status_controller` (subscribes to its `io_states`
topic; calls its `set_io` / `set_analog_output` services). UR flags are exposed
as digital signals and UR tool voltage as an analog (output, voltage) signal.

## Interface surface

| Kind | Name | Type | Purpose |
|------|------|------|---------|
| Topic (pub) | `io_states` | `io_controllers/IOStates` | Fixed-rate snapshot of every mapped signal. |
| Service | `set_io` | `io_controllers/SetIO` | Best-effort write of one or more outputs. |
| Action | `monitor_io` | `io_controllers/MonitorIO` | Change-driven feedback stream for one signal. |
| Service | `set_mock_behavior` | `io_controllers/SetMockBehavior` | Simulated mode only — schedule a future input change. |

Enumerated values are string constants on the messages whose values match the
internal enums exactly (`in`/`out`, `voltage`/`current`).

### `io_states` (topic)

Publishes a full normalized `IOStates` snapshot at a fixed rate: a
`std_msgs/Header` plus a `DigitalIO[]` and an `AnalogIO[]`. Each `DigitalIO`
carries `io_name`, `direction` (`in`/`out`), and a boolean `state`; each
`AnalogIO` adds a `domain` (`voltage`/`current`) and a `float32 state`.

### `set_io` (service)

Best-effort write. Targets are resolved by `io_name` (the direction/domain
fields in the request are advisory). Every valid entry is attempted;
`success` is true only if all succeeded, otherwise `message` summarizes the
problem and `failed_io_names` lists the entries that could not be set. Inputs
are never writable and are reported as failures.

### `monitor_io` (action)

Watches a single `io_name` and emits one feedback message on every value change
(change-driven, not rate-driven). An unknown `io_name` is **rejected** (never
accepted). Both inputs and outputs can be watched (watching an output confirms
commanded state), and multiple concurrent goals — including several on the same
signal — are allowed. The goal is long-running: it stays active until cancelled
or the node deactivates. With `emit_initial` true (the default), the server
sends one feedback with the current value immediately on acceptance.

### `set_mock_behavior` (service, simulated only)

Scripts a single **input** signal in simulated mode: schedules one discrete
state change to apply `delay_seconds` from now (0 = immediate). Calls append to
a per-signal queue. Rejected (with a reason) if the node is not simulated, the
`io_name` is unknown, the target is an output, or `delay_seconds < 0`. When the
change fires, the mock updates the cached value and invokes the change callback,
so `monitor_io` feedback and the next `io_states` snapshot reflect it.

## Configuration

The node reads a YAML mapping via the `config_file` parameter (absolute, or
resolved relative to `share/io_controllers/config`), loaded and validated once
during the lifecycle `configure` transition. A sample UR mapping ships at
[config/ur10.yaml](./config/ur10.yaml).

- `device_type` and `publish_rate_hz` form the device-agnostic envelope
  validated by the node; the opaque `device` block is driver-defined, so each
  vendor can express its mapping differently.
- Every `io_name` must be **globally unique** across all categories and both
  digital and analog signals; a duplicate fails `configure`.
- An unknown `device_type` fails `configure` with a clear message.
- Each mapping entry may carry an optional `default` (digital → bool,
  analog → float; omitted → `false` / `0.0`). It seeds the initial cached value
  and is the starting value for every signal in simulated mode.
- The config is not hot-reloaded; restart the node to apply changes.

## Running

Sim-vs-real is chosen at the **launch layer**; the node code never branches on
it. `gpio_controller` is a managed lifecycle node and starts `unconfigured`;
drive it through its transitions with the lifecycle CLI after launch.

```bash
# Simulated (device-agnostic MockIODriver); also advertises set_mock_behavior.
ros2 launch io_controllers gpio_controller.launch.py simulated:=true

# Real hardware (default). Point config_file at your device mapping.
ros2 launch io_controllers gpio_controller.launch.py \
    simulated:=false config_file:=/abs/path/to/your_device.yaml
```

Launch arguments: `simulated`, `config_file` (defaults to the packaged
`ur10.yaml`), `use_sim_time`, `namespace`, and `node_name`. After launching,
drive the lifecycle transitions:

```bash
ros2 lifecycle set /gpio_controller configure
ros2 lifecycle set /gpio_controller activate
```

## Testing

Unit and integration tests are `pytest`-based and mock all hardware, so no
robot is required:

```bash
colcon build --symlink-install --packages-select io_controllers
source install/setup.bash
python -m pytest src/io_controllers/tests/ -v
```
