# rfid_scanner

RFID scanner drivers from different vendors behind one **identical,
drop-in-replaceable ROS 2 interface**, plus the Gazebo Harmonic RFID
reader/tag system plugins used to simulate them.

| Driver node | Device | Transport |
|---|---|---|
| `simatic_rf240r_ascii` | Siemens SIMATIC RF240R, article no. 6GT2821-4AC40 | RS-232, RF200 ASCII protocol |

Every driver runs either against real hardware or against the Gazebo RFID
simulation (`simulated` parameter). Consumers do not need to know which.

---

## ROS interface (common to all drivers)

All names are node-relative (`~/…`).

| Name | Type | Notes |
|---|---|---|
| `~/tag_detected` | `std_msgs/msg/Bool` | QoS `RELIABLE`, `TRANSIENT_LOCAL`, `KEEP_LAST` depth 1 (latched). `true` when a usable tag is detected. Published on activation, then only on change. `false` is published right before the publisher is destroyed (deactivate, fatal self-transition, shutdown). |
| `~/read_data` | `rfid_scanner/srv/ReadData` | Read `length` bytes from `start_address`. |
| `~/write_data` | `rfid_scanner/srv/WriteData` | Write `data` starting at `start_address`. |

```
# ReadData.srv
uint32 start_address
uint32 length
---
bool success
string error_message
uint8[] data

# WriteData.srv
uint32 start_address
uint8[] data
---
bool success
string error_message
```

Both services validate every request identically in hardware and simulation.
Each check below fails the request with `success=false` and a descriptive
`error_message`. No exception crosses the service boundary.

1. no tag detected → `no tag detected`
2. `length == 0` / empty `data` → `requested size must be > 0`
3. size > 248 bytes → `requested size N exceeds maximum of 248 bytes`
4. (simulation only) tag detected but its payload not received yet → `tag data not available yet`
5. `start_address + size > tag memory size` → `out of bounds: …`
6. driver / transport errors (see [Error mapping](#error-mapping))

---

## Node `simatic_rf240r_ascii`

### Parameters

All parameters are read once in `on_configure`. Invalid values make
`configure` fail.

| Parameter | Type | Default | Used by | Notes |
|---|---|---|---|---|
| `simulated` | bool | `true` | both | `true` → Gazebo-simulated reader |
| `device_name` | string | `simatic_rf240r` | sim | Namespaces the Gazebo topics/service. Must equal the xacro `device_name`. Pattern `^[A-Za-z][A-Za-z0-9_]*$`. |
| `serial_port` | string | `/dev/ttyUSB0` | HW | |
| `baud_rate` | int | `19200` | HW | `19200`, `57600` or `115200` |
| `parity` | string | `odd` | HW | `odd`, `even` or `none` |
| `stop_bits` | int | `1` | HW | `1` or `2` |
| `response_timeout_sec` | double | `1.0` | both | Maximum wait for a command acknowledgement (sim: gz write service call) |
| `heartbeat_period_sec` | double | `1.0` | HW | `SLG-STATUS` heartbeat period |

### Lifecycle

The node is **not** auto-started; an external manager drives the transitions.

| Transition | Actions |
|---|---|
| `configure` | Validate parameters, construct (but do not connect) the reader. |
| `activate` | Create `~/tag_detected`, `~/read_data`, `~/write_data`; initialise the reader; publish the initial `tag_detected`. On failure everything created so far is released and the node stays *inactive*. |
| `deactivate` | Publish `false`, destroy publisher and services, de-initialise the reader (close port, stop threads/timers, drop subscriptions). |
| `cleanup` | Drop the reader and configuration. |
| `shutdown` | Release whatever exists, from any state. |

**Fatal errors** reported while *active* trigger a deferred, idempotent
`deactivate` → `cleanup` self-transition, so the node ends in *unconfigured*.
Re-run `configure` → `activate` to recover.

```bash
ros2 launch rfid_scanner simatic_rf240r_ascii.launch.py simulated:=false serial_port:=/dev/ttyUSB0
ros2 lifecycle set /simatic_rf240r_ascii configure
ros2 lifecycle set /simatic_rf240r_ascii activate
ros2 topic echo --qos-durability transient_local /simatic_rf240r_ascii/tag_detected
ros2 service call /simatic_rf240r_ascii/read_data rfid_scanner/srv/ReadData "{start_address: 0, length: 16}"
ros2 service call /simatic_rf240r_ascii/write_data rfid_scanner/srv/WriteData "{start_address: 0, data: [1, 2, 3]}"
ros2 lifecycle set /simatic_rf240r_ascii deactivate
```

### Launch file `simatic_rf240r_ascii.launch.py`

| Argument | Default |
|---|---|
| `simulated` | `true` |
| `device_name` | `simatic_rf240r` |
| `node_name` | `simatic_rf240r_ascii` |
| `serial_port` | `/dev/ttyUSB0` |
| `baud_rate` | `19200` |
| `parity` | `odd` |
| `stop_bits` | `1` |
| `response_timeout_sec` | `1.0` |
| `heartbeat_period_sec` | `1.0` |

`use_sim_time` is set to the value of `simulated`. In simulation a
`ros_gz_bridge` named `<node_name>_gz_bridge` is also started. It bridges
`/<device_name>/detected` (`std_msgs/Bool`) and `/<device_name>/tag_data`
(`ros_gz_interfaces/Dataframe`) from Gazebo to ROS. Writes do not go through
the bridge (see [Simulation](#simulation)).

---

## Real hardware: SIMATIC RF240R (RS-232 ASCII)

### Wiring and serial notes

- Only the **RS-232 ASCII variant 6GT2821-4AC40** is supported. The 3964R
  variants (-4AC10/-4AC11) are not.
- The RF240R has no USB interface. A USB-RS232 adapter simply shows up as a
  serial port (e.g. `/dev/ttyUSB0`). The container user needs access to it
  (e.g. membership in the `dialout` group).
- The **serial settings must match the settings stored on the reader**. The
  node never sends `SET-RS232`. Factory default: **19200 Bd, 8 data bits, odd
  parity, 1 stop bit**, no handshake.
- On every activation the reader is **reset** (`RESET`: "with presence" mode,
  single tag, red LED reset, ISO Ftim). With a working link the LED then turns
  green.
- The reader identity (`SLG-STATUS` mode 1) is logged on activation. A hardware
  type other than RF240R (`0x33`) or reported serial settings that differ from
  the configuration only produce a warning.

### Behaviour

- **Tag detection** comes from asynchronous `ANW-MELD` presence messages. On
  arrival the node reads `MDS-STATUS` (UID, tag type, user memory size).
  `tag_detected` becomes `true` only after that succeeds. A tag reporting a
  memory size of 0 is treated as unusable.
- **Heartbeat**: `SLG-STATUS` mode 0 every `heartbeat_period_sec`. It also
  re-syncs presence if an event was missed. It is skipped while another
  command is in flight. A **single** missed heartbeat is fatal.
- **Read/write**: one `READ`/`WRITE` telegram (1..248 bytes) per request,
  serialised with all other commands.
- A request that times out is a normal failure. Its late acknowledgement is
  discarded: before the next command the node waits (via `SLG-STATUS`) until
  the reader reports no active command.

### Error mapping

| Reader status (`error_Moby`) | Service result |
|---|---|
| `0x00` | success |
| `0x01` presence error | failure; presence is re-synced |
| `0x0D` address error | failure `out of bounds: reader reported address error (0x0D)` |
| `0x05`, `0x06`, `0x0C`, `0x13`, `0x15`, `0x19`, `0x1E`, `0x1F` | failure with the reader's error text and code |
| unknown codes | failure `unknown reader error 0xNN` |
| `0x14`, `0x18`, `0x1C` | failure **and fatal** self-transition |
| no acknowledgement within `response_timeout_sec` | failure (not fatal) |

Other **fatal** errors: serial I/O error or disappearing port, heartbeat
timeout, malformed frames (bad hex, wrong length, wrong echo) and
acknowledgements for commands that were not sent.

---

## Simulation

In simulation the node uses the merged Gazebo `RfidReader` plugin attached
by the [xacro macro](#xacro--srdf-macros):

| Gazebo name | Type | Use |
|---|---|---|
| `/<device_name>/detected` | `gz.msgs.Boolean` → bridged `std_msgs/Bool` | `tag_detected` follows this topic |
| `/<device_name>/tag_data` | `gz.msgs.Dataframe` → bridged `ros_gz_interfaces/Dataframe` | tag id (`src_address`) and full payload (`data`); memory size = payload length |
| `/<device_name>/write_tag` | gz-transport service, `gz.msgs.Dataframe` → `gz.msgs.StringMsg` | writes, called directly via gz-transport |

- **Read** slices the cached payload.
- **Write** calls the gz write service (timeout `response_timeout_sec`). On
  success the node updates its payload cache optimistically, so an immediate
  read returns the new bytes. A `tag_data` message already in flight may
  briefly carry the old content until the next plugin publication (≤ 100 ms at
  10 Hz).
- The plugin validates writes against the detection result of its last
  throttled update (at most 100 ms old at 10 Hz).
- A write fails if the plugin has no write service, the call times out (for
  example while the simulation is paused) or the plugin rejects it.
- Writes last for the lifetime of the tag entity. A re-spawned tool model gets
  its SDF payload again.
- Silence on the topics is not treated as a connection loss. Only unexpected
  internal errors (e.g. gz-transport node creation failing) are fatal.

---

## XACRO / SRDF macros

```xml
<xacro:include filename="$(find rfid_scanner)/urdf/rfid_scanner.urdf.xacro"/>

<xacro:rfid_scanner
  device_type="simatic_rf240r"
  device_name="station_rfid"
  simulated="$(arg use_sim)"
  parent="station"
  tf_prefix=""
  mount_origin_xyz="0.2 0.1 0.0"
  mount_origin_rpy="0 0 0"/>
```

| Arg | Default | Purpose |
|---|---|---|
| `device_type` | `simatic_rf240r` | Selects meshes, inertial, link offsets and plugin parameters. Unknown values abort the expansion. |
| `device_name` | `simatic_rf240r` | Link/joint prefix and namespace of the Gazebo topics/service. Must equal the node's `device_name`. |
| `simulated` | `true` | Adds the scanner link and the `RfidReader` plugin. |
| `parent` | `station` | Parent link. |
| `tf_prefix` | `''` | Prefix applied to `parent`. |
| `mount_origin_xyz` / `mount_origin_rpy` | `0 0 0` | Pose of `<device_name>_device_origin` relative to the parent. |

Generated frames (`N` = `device_name`, all fixed joints):

- `N_device_origin`: the mounting frame + the housing, with meshes from `model/simatic/`.
- `N_scanner_link` (simulated only): the antenna face the `RfidReader` is
  attached to. The plugin uses `range` 0.065 m, `update_rate` 10 Hz,
  `detected_topic` `/N/detected`, `content_topic` `/N/tag_data`,
  `write_service` `/N/write_tag` and `reader_id` `N`.

Dimensions, inertia and offsets are **placeholders** (marked `PLACEHOLDER` in
the xacro), as are the meshes in `model/simatic/meshes/`.

```xml
<xacro:include filename="$(find rfid_scanner)/srdf/rfid_scanner.srdf.xacro"/>
<xacro:rfid_scanner_srdf parent="station" device_name="station_rfid" tf_prefix=""/>
```

---

## Gazebo plugins (`rfid_sim_systems`)

Two gz-sim (Harmonic) system plugins and a shared `RfidTagData` ECM component,
built into the library **`rfid_sim_systems`** (namespace `autofactory::rfid`).
An environment hook adds the library directory to `GZ_SIM_SYSTEM_PLUGIN_PATH`,
so `<plugin filename="rfid_sim_systems" …>` resolves after sourcing the
workspace.

### `autofactory::rfid::RfidTag`

Attaches a fixed payload to a link.

| SDF element | Default | Meaning |
|---|---|---|
| `<link>` | canonical link | Link the tag is attached to |
| `<id>` | entity name | Tag id, published as `Dataframe.src_address` |
| `<data encoding="hex\|base64\|plain">` | empty, encoding `plain` | Payload bytes |

### `autofactory::rfid::RfidReader`

On each throttled update it finds the nearest tag within `<range>` and
publishes a detection flag and the tag payload.

| SDF element | Default | Meaning |
|---|---|---|
| `<link>` | canonical link | Reader link; its world position is the centre of the detection sphere |
| `<range>` | `0.1` | Detection radius [m]; values ≤ 0 fall back to 0.1 |
| `<update_rate>` | `0` (every step) | Publication rate [Hz] |
| `<detected_topic>` | `/<scoped name>/rfid/detected` | `gz.msgs.Boolean` detection flag |
| `<content_topic>` | `/<scoped name>/rfid/content` | `gz.msgs.Dataframe`: `src_address` = tag id, `dst_address` = reader id, `data` = payload (empty when no tag) |
| `<reader_id>` | entity name | Placed into `Dataframe.dst_address` |
| `<write_service>` | *(absent)* | **Optional.** gz service accepting writes to the detected tag. Without it the reader behaves exactly as before. |
| `<write_timeout>` | `0.5` | Seconds the write service waits for the write to be applied; values ≤ 0 fall back to 0.5 |

Write service contract:

- Request `gz.msgs.Dataframe`: `src_address` = expected tag id (rejected if a
  different tag or none is detected), `dst_address` = decimal start address,
  `data` = bytes.
- Response `gz.msgs.StringMsg`: empty = success, otherwise the error text.
- The write is applied in the simulation loop (`PreUpdate`) to the tag's
  `RfidTagData` component, so every reader sees it. Writes beyond the payload
  size are rejected (tags never grow). While the simulation is paused, the call
  times out.

```bash
gz service -s /simatic_rf240r/write_tag --reqtype gz.msgs.Dataframe --reptype gz.msgs.StringMsg \
  --timeout 1000 --req 'src_address: "tag-A17", dst_address: "0", data: "\x01\x02"'
```

### Demo

```bash
gz sim -r $(ros2 pkg prefix rfid_scanner)/share/rfid_scanner/worlds/rfid_demo.sdf
ros2 launch rfid_scanner rfid_bridge.launch.py
ros2 topic echo /rfid/reader0/content
```

---

## Build

```bash
colcon build --symlink-install --packages-select rfid_scanner
source install/setup.bash
```

Python dependency: `pyserial` (pinned in the workspace `requirements.txt`).
