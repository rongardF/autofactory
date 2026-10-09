# `tools_manager`

ROS 2 (Jazzy) package that **orchestrates automatic tool changing** for the
robot cell. It knows which tools live in which tool-rack slots, which tool (if
any) is on the robot's tool-mount, and it drives the full mount/unmount
choreography: moving the robot, operating the quick-release lock, and keeping
both the **MoveIt 2 planning scene** and the **Gazebo world** in sync with
reality.

This document describes the behaviour **as implemented in the code**, so it can
be used to verify that the package does what is intended.

---

## Nodes (executables)

| Node | File | Role |
|------|------|------|
| `tools_manager` | `tools_manager.py` | Orchestrator. Owns the mount/unmount actions, launches endtool nodes, manages world/planning-scene models. |
| `tool_rack` | `tool_rack.py` | Publishes which tools are in which rack slots. |
| `tool_mount` | `tool_mount.py` | Publishes the currently-mounted tool and exposes the quick-release `lock` service. |

All three are `rclpy` **LifecycleNode**s spun with a `MultiThreadedExecutor`.

### Supporting package

Simulation relies on **`rfid_sim`**, a C++ gz-sim (Harmonic) system plugin pair:
- `autofactory::rfid::RfidTag` — attaches a fixed hex payload (the tool's RFID
  tag) to a link via an `RfidTagData` ECM component.
- `autofactory::rfid::RfidReader` — on each throttled `PostUpdate`, finds the
  nearest tag within `<range>` and publishes a `Boolean` "detected" flag and a
  `Dataframe` with the tag bytes. These are bridged to ROS as
  `/<scanner>/tag_data` (`ros_gz_interfaces/Dataframe`).

So in simulation, a slot/mount "sees" a tool because an RFID reader on that link
detects the tool model's RFID tag — exactly how a real reader would.

---

## Configuration (`config/tools_manager_config.yaml`)

Validated by `ToolsManagerConfigDTO` (Pydantic v2). Two sections:

### `slots` — the static rack definition
Each entry is a `ToolSlotDTO`:
- `index` — rack slot number.
- `tool_sn` — serial expected in that slot (`null` ⇒ empty slot).
- `metadata` — `{ launch_file, model }` (which endtool launch file + model dir).
- `parameters` — arbitrary endtool node parameters (must include `node_name`);
  values limited to str/float/int/bool/list.

Validators enforce: no duplicate serials or indices, and if `tool_sn` is `null`,
`metadata`/`parameters` are forced to `null`.

### `simulation_setup` — what to spawn at boot in sim
- `tool_rack` — per slot index, a hex `tag_data` RFID payload (`null` ⇒ empty).
- `tool_mount` — optional hex `tag_data` for a tool already on the mount.

Validators enforce: simulation entries match the declared slot indices 1:1, and
no duplicate serials (serial is decoded from bytes `0:15` of the tag payload).

### Tool identity encoding (`ToolInfoDto`)
A tool's identity is a **96-byte RFID payload** = six 16-byte, null/space-padded
fields: `serial, tool_type, tool_part_number, tool_part_revision,
material_part_number, material_part_revision`. `ToolInfoDto.from_tag_data` /
`to_tag_data` convert between the hex payload and the structured DTO. Each tool
also derives three frame names from its slot index:
- `slot<index>_slide_in_link`
- `slot<index>_attached_link`
- `slot<index>_lifted_link`

---

## `tool_rack` node

Publishes a latched (`TRANSIENT_LOCAL`) `~/slots` topic (`tools_manager/Slots`)
at `slots_update_rate` Hz.

- **Parameters:** `simulated`, `tools_manager_config_file`, `parent_frame_id`
  (default `station`), `slots_update_rate` (Hz).
- **`on_configure`:** reads the config file, builds `ToolRackNodeConfigDTO`,
  creates the `~/slots` publisher, and picks `SimulatedRackController` or
  `HardwareRackController`.
- **`on_activate`:** `controller.setup()` then a timer that publishes slot data.
- **Simulated controller:** subscribes to `/slot<index>_rfid_scanner/tag_data`
  for each slot; each `Dataframe` is parsed into a `ToolInfoDto` (empty payload ⇒
  slot empty). `get_slots_data()` returns a `SlotsDto` of expected vs.
  currently-detected tools. On setup it warns if detected tools don't match the
  configured slots.
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
`tool_rack_node_name`, `movement_controller_node_name`, `tools_manager_config_file`.

### Collaborators it builds
- **`GazeboWorldManager`** (sim only) — spawns/removes/re-mates tool **models**
  in the live Gazebo world over `ros_gz_bridge`.
- **`Moveit2WorldManager`** — manages tool **collision objects** in the MoveIt 2
  planning scene.
- **`NodeStateManager`** × tool_rack / tool_mount / each endtool — drives the
  lifecycle transitions and parameters of other nodes **by name**, over their
  standard `change_state` / `get_state` / `set_parameters` / `get_parameters`
  services.
- **`Ros2Launcher`** — launches and shuts down endtool launch files as child
  processes.

### `on_configure`
Reads the config file, builds `ToolsManagerNodeConfigDTO`, and (in sim) creates
the Gazebo manager; always creates the MoveIt manager, the two node-state
managers, and the launcher.

### `on_activate` — bring the cell up
1. Sanity-check that all collaborators exist.
2. Configure + activate the `tool_rack` and `tool_mount` nodes.
3. Subscribe (latched) to `tool_rack/slots` and `tool_mount/tool_mounted`.
4. Determine the installed/mounted tool set:
   - **Sim:** read directly from `simulation_setup` (tag data → `ToolInfoDto`);
     index `-1` marks a tool on the mount.
   - **Hardware:** poll the latched topics up to 10× (1 s apart) for slot +
     mount info.
5. For **each tool on the rack**: look up its parameters/metadata, **launch its
   endtool node** (`Ros2Launcher.launch(...)` with `mounted:=false`,
   `tool_rack_link:=slot<index>_attached_link`), register a `NodeStateManager`,
   configure that endtool node, then **spawn its model** into the MoveIt scene
   and (in sim) the Gazebo world, attached to the slot link.
6. If a tool is **on the mount**, do the same with `mounted:=true`, attaching to
   `tool_mount_tcp`.
7. Create the `tool_mount/lock` service client, the movement-controller
   `ExecuteTrajectory` action client, and the two action servers
   `~/mount_tool` and `~/unmount_tool`.
8. On `ActivationFailedException`, unwind everything created so far and return
   `FAILURE`.

### Actions

Both are `tools_manager` actions with goal `string tool_sn` and result
`bool success, string message`.

#### `~/mount_tool` (pick a tool off the rack)
Serialized by a non-blocking `RLock` (concurrent mount/unmount is rejected).
Pre-checks: nothing already mounted, slots known, collaborators initialized, the
requested serial actually on the rack. Then:

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
    (unconfigure → set `mounted=true` → configure).
11. `goal_handle.succeed()`.

Any failed step aborts the goal with an explanatory `message`.

#### `~/unmount_tool` (put the tool back)
Mirror image, also serialized. Pre-checks: a tool is mounted and its home slot
is empty. Sequence: PTP → `lifted`, allow collisions, LIN → `attached`, **re-mate
model back to the rack slot link**, `lock` unlock, wait up to 10 s for the rack
to re-detect the tool, LIN → `slide_in`, re-disable collisions,
`_endtool_mounted(tool_sn, False)`, succeed.

### Movement + service plumbing
- `_call_action` — `wait_for_server`, `send_goal_async`, polls the future via
  the node clock (honours `use_sim_time`), waits indefinitely for the result,
  and only returns `True` if status is `SUCCEEDED` **and** `result.success`.
- `_call_service` — `wait_for_service`, `call_async`, polled with a timeout.
- `_wait_for_future` — polls `future.done()` without nested spinning (the future
  is serviced by another executor thread).

---

## World/scene management

### `Moveit2WorldManager` (planning scene)
Represents each tool as an **attached collision object** on exactly one link.
- `spawn_model` — loads the tool STL via `trimesh` into a `shape_msgs/Mesh`,
  attaches it to the given link (`touch_links` = attach link + configured extras).
- `attach_to_link` — detach-then-attach transfer between links (reuses geometry).
- `delete_model` — detach then remove the world object.
- `allow_collisions(model, allowed, tool_mount_link, slot_link)` — reads the
  Allowed Collision Matrix, pairwise toggles the three entities (tool,
  tool-mount link, slot link), and re-applies the scene. Called `True` before a
  slide-in/out maneuver and `False` afterwards.
- A 1 mm `+x` offset is added to attached objects to avoid self-collision with
  the link origin.

### `GazeboWorldManager` (live Gazebo world, sim only)
Talks to Gazebo purely through `ros_gz_bridge` (never imports `gz.*`).
- `spawn_model` — expands the tool XACRO to SDF (passing `model_name`,
  `station_model_name`, `tool_mount_link`, `tool_rack_link`, `tag_data`), looks
  up the target link's world pose via TF, and spawns via the world `create`
  service. Because both detachable joints **start attached**, it then detaches
  the redundant joint so the tool is welded to the requested link only.
- `attach_to_link` — a **transfer** that attaches the destination joint first,
  then detaches the source, so the tool is never held by zero joints.
- `delete_model` — best-effort detach, then the world `remove` service.
- Attach/detach are confirmed by subscribing to the joint's `.../state` topic
  before publishing the `.../attach|detach` request, then waiting for the
  expected `attached`/`detached` state (with timeout). It also waits for the
  bridge subscriber to match before publishing so requests aren't dropped.
- An internal `_attached_link_lookup` tracks which link each model is on, so
  `attach_to_link` only needs the destination.

> `GazeboWorldManager._generate_model_path` (and the MoveIt one) resolve the
> model file from the tool's metadata (`ToolMetadataDto.model`) against the
> installed `endtools` package share directory
> (`<endtools_share>/model/<model>/...`). Both managers receive the
> `tools_manager_config` so they can look up the metadata by tool serial number.

---

## `Ros2Launcher`

Launches each endtool launch file as an independent child process
(`ros2 launch ...`, no shell, argument-injection safe), in its own process group.
- A per-launch daemon thread watches the process and flips `LaunchDto.running`
  to `False` when it exits.
- `shutdown` escalates `SIGINT → SIGTERM → SIGKILL` to the process group, like
  Ctrl-C in the terminal. `shutdown_all` tears every launch down on node teardown.
- Scalar/list launch args are serialized to `key:=value` (`bool` → `true`/`false`).

---

## Topics / interfaces summary

| Interface | Type | Direction |
|-----------|------|-----------|
| `tool_rack/slots` | `tools_manager/Slots` (latched) | rack → manager |
| `tool_mount/tool_mounted` | `tools_manager/ToolInfo` (latched) | mount → manager |
| `tool_mount/lock` | `std_srvs/SetBool` | manager → mount |
| `~/mount_tool`, `~/unmount_tool` | `tools_manager/MountTool`, `UnmountTool` | clients → manager |
| `<movement_controller>/execute_trajectory` | `movement_controller/ExecuteTrajectory` | manager → mover |
| `/world/<world>/{create,remove,set_pose}` | `ros_gz_interfaces/*` | manager → Gazebo (sim) |
| `/<link>/<model>/{attach,detach,state}` | `std_msgs/Empty` / `String` | manager ↔ Gazebo joints (sim) |
| `/slot<i>_rfid_scanner/tag_data`, `/tool_mount_rfid_scanner/tag_data` | `ros_gz_interfaces/Dataframe` | Gazebo → rack/mount (sim) |

---

## Launch / run

```bash
colcon build --symlink-install --packages-select rfid_sim endtools tools_manager
source install/setup.bash

ros2 launch tools_manager tools_manager_launch.py simulated:=true
```

`tools_manager_launch.py` starts `tool_mount`, `tool_rack`, and `tools_manager`,
and — only in sim — a `ros_gz_bridge` that bridges every slot/mount RFID scanner
`tag_data` topic and the Gazebo `create`/`set_pose`/`remove` world services.

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
