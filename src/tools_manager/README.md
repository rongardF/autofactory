# `tools_manager`

ROS 2 (Jazzy) package that **orchestrates automatic tool changing** for the
robot cell. It knows which tools live in which tool-rack slots, which tool (if
any) is on the robot's tool-mount, and it drives the full mount/unmount
choreography: moving the robot, operating the quick-release lock, and keeping
both the **MoveIt 2 planning scene** and the **Gazebo world** in sync with
reality.

---

## Nodes (executables)

| Node | File | Role |
|------|------|------|
| `tools_manager` | `tools_manager.py` | Orchestrator. Owns the mount/unmount actions, configures the endtool nodes started by launch, manages world/planning-scene models. |
| `tool_rack` | `tool_rack.py` | Publishes which tools are in which rack slots. |
| `tool_mount` | `tool_mount.py` | Publishes the currently-mounted tool and exposes the quick-release `lock` service. |

All three are `rclpy` **LifecycleNode**s spun with a `MultiThreadedExecutor`.

### Supporting package

Simulation relies on the C++ gz-sim (Harmonic) system plugin pair that ships
with **`rfid_scanner`** (library `rfid_sim_systems`):
- `autofactory::rfid::RfidTag` — attaches a fixed hex payload (the tool's RFID
  tag) to a link via an `RfidTagData` ECM component.
- `autofactory::rfid::RfidReader` — on each throttled `PostUpdate`, finds the
  nearest tag within `<range>` and publishes a `Boolean` "detected" flag and a
  `Dataframe` with the tag bytes. These are bridged to ROS as
  `/<scanner>/tag_data` (`ros_gz_interfaces/Dataframe`).

So in simulation, a slot/mount "sees" a tool because an RFID reader on that link
detects the tool model's RFID tag — exactly how a real reader would.

---

## Configuration (ROS parameters only)

The nodes never read a configuration file; everything is a ROS parameter.
Launch supplies the values (see [Launch / run](#launch--run)).

### `tools.*` — expected tools and physical rack slots
Passed identically to `tool_rack` and `tools_manager`, validated by
`ToolsManagerConfigDTO` (Pydantic v2). The three arrays are associated **by
position**:

| Parameter | Type | Meaning |
|-----------|------|---------|
| `tools.index` | int array | Every physical rack slot index (the slot id, not an array position) |
| `tools.tool_sn` | string array | Serial expected for each slot |
| `tools.node_name` | string array | Endtool node name belonging to each serial |

```yaml
tools:
  index: [1, 2, 3]
  tool_sn: ["abc123", "", "def123"]
  node_name: ["dispenser_abc123", "", "dispenser_def123"]
```

Rules: all arrays have the same length (checked before anything is built, never
truncated); every physical slot is listed, an unassigned slot has an empty
serial **and** an empty node name (they come together); slot indices, non-empty
serials and non-empty node names are unique. A tool's index is its **home
slot**, also while it sits on the tool mount. Above, tool `abc123` may be in
slot 1 or on the mount, never in slot 2 or 3.

### `simulation.*` — initial simulated inventory (`tools_manager` only)
| Parameter | Type | Meaning |
|-----------|------|---------|
| `simulation.tool_rack.index` | int array | Slot indices of the initial rack inventory |
| `simulation.tool_rack.tag_data` | string array | Hex-encoded RFID payload per slot (`""` ⇒ empty slot) |
| `simulation.tool_mount` | string | Hex-encoded RFID payload of the mounted tool (`""` ⇒ empty mount) |

The arrays must have the same length, indices must be unique and exist in
`tools.index`, and every payload is decoded and validated through
`ToolInfoDto.from_hex_tag_data`. They are only read when `simulated` is true.

`config/tools_manager_config.yaml` is a ROS parameter file holding these
values; the launch file passes it to `tools_manager`.

### Tool identity encoding (`ToolInfoDto`)
A tool's identity is a **96-byte RFID payload** = six 16-byte, null/space-padded
fields: `serial, tool_type, tool_part_number, tool_part_revision,
material_part_number, material_part_revision`. `ToolInfoDto.from_hex_tag_data` /
`to_tag_data` convert between the hex payload and the structured DTO; invalid
hex, a short payload, an empty serial or an unknown tool type is rejected.
Each tool also derives three frame names from its slot index:
- `slot<index>_slide_in_link`
- `slot<index>_attached_link`
- `slot<index>_lifted_link`

---

## `tool_rack` node

Publishes a latched (`TRANSIENT_LOCAL`) `~/slots` topic (`tools_manager/Slots`)
at `slots_update_rate` Hz.

- **Parameters:** `simulated`, `tools.*`, `parent_frame_id`
  (default `station`), `slots_update_rate` (Hz).
- **`on_configure`:** reads the `tools.*` parameters, builds `ToolRackNodeConfigDTO`,
  creates the `~/slots` publisher, and picks `SimulatedRackController` or
  `HardwareRackController`.
- **`on_activate`:** `controller.setup()` then a timer that publishes slot data.
- **Simulated controller:** subscribes to `/slot<index>_rfid_scanner/tag_data`
  for each physical slot (assigned or not); each `Dataframe` is parsed into a
  `ToolInfoDto` (empty payload ⇒ slot empty). `get_slots_data()` returns a
  `SlotsDto` of expected vs. currently-detected tools; validating the placement
  is the tools manager's job.
- **Hardware controller:** present but not the focus of the sim path.

## `tool_mount` node

Publishes a latched `tool_mounted` topic (`tools_manager/ToolInfo`) at
`mounted_publish_rate` Hz and exposes a `lock` service (`std_srvs/SetBool`).

- **Parameters:** `simulated`, `parent_frame_id` (default `tool0`),
  `mounted_publish_rate`.
- **`on_activate`:** `controller.setup()`, a timer (`_check_mounted_tool`) that
  publishes the mounted-tool info, and the `lock` service.
- **`lock` service:** `data=True` ⇒ close (lock), `data=False` ⇒ open (unlock);
  delegates to `controller.lock_closed(...)`.
- **Simulated controller:** subscribes to `/tool_mount_rfid_scanner/tag_data`;
  parses the `Dataframe` into the mounted `ToolInfoDto` (empty ⇒ nothing
  mounted). `lock_closed()` is a no-op returning `True` (there is no physical
  lock to drive in sim).

---

## `tools_manager` node — the orchestrator

### Parameters
`simulated`, `world_name`, `station_model_name`, `tool_mount_node_name`,
`tool_rack_node_name`, `movement_controller_node_name`, `tools.*`, and — in
simulation — `simulation.*`.

### Collaborators it builds
- **`ToolsWorldClientAdapter`** — resolves tool geometry and XACRO mappings,
  then uses the shared `world_manager` clients for Gazebo (simulation only)
  and MoveIt planning-scene operations.
- **`NodeStateManager`** × tool_rack / tool_mount / each endtool — drives the
  lifecycle transitions and parameters of other nodes **by name**, over their
  standard `change_state` / `get_state` / `set_parameters` / `get_parameters`
  services.

### Ownership
| Owner | Responsibility |
|-------|----------------|
| ROS 2 launch | Endtool and pattern executor process lifetime |
| Tools manager | Endtool configuration, mounted-state reconfiguration, tool world models |
| Pattern executor | Activation/deactivation of its endtool during its own lifecycle |
| External orchestrator | Startup ordering; keeping the executor inactive around tool changes and before the manager is deactivated |

The manager never starts or stops processes and never drives executor
lifecycles. Its endtools stay `inactive` after activation; an unmounted endtool
cannot be activated for dispensing.

### `on_configure`
Reads and validates the parameters into `ToolsManagerNodeConfigDTO` (failing on
any inconsistency), then creates the Gazebo client (sim only), the MoveIt client
and the rack/mount node-state managers.

### `on_activate` — bring the cell up
1. Sanity-check that all collaborators exist, and clean up anything left by an
   earlier failed attempt.
2. Check the world-manager services, then configure + activate the `tool_rack`
   and `tool_mount` nodes.
3. Subscribe (latched) to `tool_rack/slots` and `tool_mount/tool_mounted`.
4. Obtain the initial inventory:
   - **Sim:** from `simulation.*`; index `-1` marks the tool on the mount.
   - **Hardware:** poll up to 10× (1 s apart) until a rack message **and** a
     mount message have arrived. An empty mount message is a valid answer and is
     distinguished from "nothing received yet".
5. **Validate the inventory** against `tools.*` before touching any endtool.
   Activation fails with an actionable message listing every problem when a
   configured tool is absent from rack and mount, a detected serial is unknown
   (including any tool in an unassigned slot), a known tool sits in a slot other
   than its home slot, or a serial is detected more than once. A tool on the
   mount is valid although its home slot is empty.
6. **Associate** each detected serial with its mapped endtool node: the node
   must be reachable and `unconfigured`, its `tool_sn` parameter must equal the
   serial, and its read-only `model` parameter must be readable.
7. For each endtool: set its `mounted` parameter from the observed location,
   **configure** it (left `inactive`), then create its models in the MoveIt
   scene and (in sim) Gazebo, attached to the home slot link or to
   `tool_mount_tcp`.
8. Create the `tool_mount/lock` service client, the movement-controller
   `ExecuteTrajectory` action client and `in_session` / `close_session` service
   clients, the two action servers `~/mount_tool` and `~/unmount_tool`, and the
   `~/get_tool_node_name` service. The manager only reports success after all of
   this completed.
9. On any failure, roll back **only what was done**: models that were created
   are removed, endtools that were configured go back to `unconfigured`
   (deactivating first if needed), rack/mount nodes that were brought up are
   returned to `unconfigured`, and interfaces are destroyed. Rollback failures
   are logged explicitly and are retried at the next activation. Endtool and
   executor processes are never touched, so activation can simply be retried.

### `on_deactivate`
Destroys the interfaces, removes the tool models, returns the endtools (and
rack/mount nodes) to `unconfigured` and releases their lifecycle clients. The
endtool and executor processes keep running, so the manager can be activated
again. The orchestrator must have deactivated the executors first.

### Actions

Both are `tools_manager` actions with goal `string tool_sn` and result
`bool success, string message`.

#### `~/mount_tool` (pick a tool off the rack)
Serialized by a non-blocking `RLock` that is taken once per goal (concurrent
mount/unmount is rejected before it can touch the running movement session) and
held for the whole operation, including the unmount of an automatic swap.
Pre-checks: slots known, collaborators initialized, the requested serial
actually on the rack. If the requested tool is already mounted the goal
succeeds immediately (`success=true`, no motion). If a **different** tool is
mounted, it is first returned to its slot (the `~/unmount_tool` sequence below,
run on the same goal handle) and then the requested tool is mounted; a failure
in either part aborts the goal with a single failed result and no rollback.
The unmount pre-checks (mounted tool's slot known and empty) run before any
motion. Then:

1. Resolve the tool's three frames (`slide_in`, `attached`, `lifted`). All robot
   moves target a **zero "unity" pose** in those frames, i.e. "make
   `tool_mount_tcp` coincide with the target frame".
2. **PTP** move `tool_mount_tcp` → `slot<n>_slide_in_link`.
3. `planner.allow_collisions(tool, True, ...)` — permit tool/mount/rack contact.
4. `tool_mount/lock` **unlock** (`SetBool False`).
5. **LIN** move (0.02 m/s) → `slot<n>_attached_link`.
6. `tool_mount/lock` **lock** (`SetBool True`).
7. Wait up to 10 s for `tool_mount/tool_mounted` to report the expected serial.
8. **Re-mate the model**: `attach_to_link(tool, 'tool_mount_tcp')` in MoveIt and
   (sim) in Gazebo.
9. **LIN** move → `slot<n>_lifted_link`; re-disable collisions.
10. `_endtool_mounted(tool_sn, True)` — reconfigure the endtool node as mounted
    (unconfigure → set `mounted=true` → configure). The executor/endtool must
    be inactive. If this fails the goal is aborted with a message saying the
    tool **was** mounted but its endtool could not be reconfigured.
11. `goal_handle.succeed()`.

Any failed step aborts the goal with an explanatory `message`.

#### `~/unmount_tool` (put the tool back)
Mirror image, also serialized. Pre-checks: a tool is mounted, the requested
`tool_sn` exactly matches its serial (case-sensitive), and its home slot is
empty. An incorrect or empty serial aborts with `success=false` and a message
identifying the requested and mounted serials, before any motion, collision
changes, lock operation, or endtool reconfiguration. During an automatic swap,
the internal unmount step explicitly targets the currently mounted serial,
not the replacement serial in the mount goal.
Sequence: PTP → `lifted`, allow collisions, LIN → `attached`, **re-mate
model back to the rack slot link**, `lock` unlock, wait up to 10 s for the rack
to re-detect the tool, LIN → `slide_in`, re-disable collisions,
`_endtool_mounted(tool_sn, False)` (a failure aborts the goal, stating that the
tool was already returned to its slot), succeed.

### Services

#### `~/get_tool_node_name` (`tools_manager/srv/GetToolNodeName`)
Only available while the node is **active**. Request: `string tool_sn`. The
tool must be on the rack (`tool_rack/slots`) or be the mounted tool
(`tool_mount/tool_mounted`); the node name comes from the configured slots
(`tools.tool_sn` / `tools.node_name`).
Response: `bool success`, `string error_message` (only set when
`success=false`), `string node_name` (the endtool node name; empty on failure).

### Movement + service plumbing
- **Movement session** — every top-level `mount_tool` / `unmount_tool` goal runs
  all its movements (including the unmount done by a tool swap) in one
  movement-controller session so no other caller can move the robot in between.
  The session (`movement_controller.services.MovementSession`) is opened by the first movement
  with a hardcoded 60 s timeout, re-extended (60 s) before a movement when less
  than 30 s remain, and always closed when the operation ends. If it cannot be
  acquired (e.g. another caller holds the lock) the movement and therefore the
  operation fails immediately, without retry.
- `_call_action` — opens/extends the movement session and sets `session_id` on the
  goal, then `wait_for_server`, `send_goal_async`, polls the future via
  the node clock (honours `use_sim_time`), waits indefinitely for the result,
  and only returns `True` if status is `SUCCEEDED` **and** `result.success`.
- `_call_service` — `wait_for_service`, `call_async`, polled with a timeout.
- `_wait_for_future` — polls `future.done()` without nested spinning (the future
  is serviced by another executor thread).

---

## World/scene management

`ToolsWorldClientAdapter` resolves each tool's collision mesh and SDF XACRO
from the tool's `model` parameter (read from the endtool node) in the installed
`endtools` share directory. It submits
collision-object changes and the tool/mount/rack ACM pairs through
`MoveIt2WorldClient`; in simulation it also submits model spawn, attachment,
and deletion requests through `GazeboWorldClient`. The adapter preserves the
tool-mount rotation, a 1 mm `+x` MoveIt offset, and the RFID `tag_data` XACRO
mapping. Gazebo world manipulation is owned by `world_manager`, not by a
per-consumer ROS-Gazebo service or detachable-joint bridge.

---

## Endtool discovery parameters

Every endtool exposes two **read-only** parameters, declared in its constructor
and set once at startup through the standard ROS parameter override mechanism
(its launch file forwards them via the node's `parameters`), so they can be read
while the node is `unconfigured`:

| Parameter | Used by |
|-----------|---------|
| `model` | The tools manager, to find the collision mesh and Gazebo model of the tool. Launch argument default for the dispenser: `dispensing_tool`. |
| `pattern_executor` | Discovery only: the node name of the executor associated with the tool. The manager does not drive that executor's lifecycle. |

---

## Topics / interfaces summary

| Interface | Type | Direction |
|-----------|------|-----------|
| `tool_rack/slots` | `tools_manager/Slots` (latched) | rack → manager |
| `tool_mount/tool_mounted` | `tools_manager/ToolInfo` (latched) | mount → manager |
| `tool_mount/lock` | `std_srvs/SetBool` | manager → mount |
| `~/mount_tool`, `~/unmount_tool` | `tools_manager/MountTool`, `UnmountTool` | clients → manager |
| `~/get_tool_node_name` | `tools_manager/GetToolNodeName` | clients → manager |
| `<movement_controller>/execute_trajectory` | `movement_controller/ExecuteTrajectory` | manager → mover |
| `<movement_controller>/in_session`, `<movement_controller>/close_session` | `movement_controller/InSession`, `CloseSession` | manager → mover |
| `/gazebo_world_manager/{spawn_models,delete_models,set_model_attachment,list_models}` | `world_manager` services | manager ↔ Gazebo (sim) |
| `/moveit2_world_manager/{apply_world_changes,attach_object,list_objects}` | `world_manager` services | manager ↔ MoveIt |
| `/slot<i>_rfid_scanner/tag_data`, `/tool_mount_rfid_scanner/tag_data` | `ros_gz_interfaces/Dataframe` | Gazebo → rack/mount (sim) |

---

## Launch / run

```bash
colcon build --symlink-install --packages-up-to tools_manager
source install/setup.bash

ros2 launch tools_manager tools_manager_launch.py simulated:=true
```

`tools_manager_launch.py` starts `tool_mount`, `tool_rack`, `tools_manager`, the
`dispensing_optimizer` shared by the dispensing executors,
the endtools of the rack with their pattern executors, and — only in sim — a
`ros_gz_bridge` for the RFID scanner `tag_data` topics of every rack slot and of
the mount. If a parent launch already starts the shared world manager, keep
`start_world_manager:=false` (the default); the endtool launches never start one.

The rack is defined statically at the top of the launch file (`_RACK_SLOTS`):
each physical slot with the endtool that lives in it. The same `tools.*` values
are passed to `tool_rack` and `tools_manager`; an unassigned slot (`None`) gets
neither an endtool nor an executor. For every endtool the launch includes
`endtools/volumetric_dispenser_launch.py` and
`pattern_executors/dispensing_pattern_executor.launch.py`, passing the same
`simulated`, `world_name` and tool identity. The executor node name is a launch
argument per instance (`pattern_executor_<tool_sn>`); its resolved value names
the executor node **and** is forwarded to the endtool launch as
`pattern_executor`. Add or change tools by editing `_RACK_SLOTS`.

The pattern executor launch lives in a package that already depends on
`endtools`, so the composition is done here (`tools_manager` depends on both)
rather than from the endtools launch, which keeps the dependency graph acyclic.

An external orchestrator must configure and activate the included world-manager
nodes before activating `tools_manager`, and keep the executors inactive around
tool changes.

Then drive the orchestrator's lifecycle and call an action:
```bash
ros2 lifecycle set /tools_manager configure
ros2 lifecycle set /tools_manager activate
ros2 action send_goal /tools_manager/mount_tool tools_manager/action/MountTool "{tool_sn: 'abc123'}"
```

---

## Observations (for correctness verification)

Discrepancies found while reading the code that affect whether the package
behaves as intended:

1. **Hardware controllers** (`tool_rack` / `tool_mount`) exist but the fully
   wired path is the simulated one.
