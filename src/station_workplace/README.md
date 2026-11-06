# station_workplace

Lifecycle nodes that manage the product present in a station workplace.

When the product carrier reports a product in place, the node identifies the
product, fetches the product and carrier meshes, attaches them to the MoveIt 2
planning scene, publishes a TF frame for the estimated product origin and
publishes the workplace state. When the product leaves, all of this is reverted.

| Node (executable) | Product data source |
|-------------------|---------------------|
| `workplace_rfid_scanner` | RFID tag, via `/<rfid_scanner_node_name>/tag_detected` and `/read_data` |
| `workplace_barcode_scanner` | Barcode label, via `/<barcode_scanner_node_name>/label_data` |

Both nodes share all behaviour except reading the product data
(`WorkplaceNodeBase` + `WorkplaceProcessService`).

## Interfaces

### Messages

| Message | Content |
|---------|---------|
| `ProductData` | `product_number`, `product_revision`, `serial_number` |
| `WorkplaceState` | `in_place`, `ProductData[<=1] product_data` (empty when no product) |
| `LabelData` | `uint8[] data` (ASCII label), `builtin_interfaces/Time stamp` |

An unidentified product is reported as `ProductData` with all fields `UNKNOWN`.

### Actions

| Action | Goal | Result | Feedback |
|--------|------|--------|----------|
| `WaitRouting` | `string routing_operation` (`injection` / `ejection`, constants `ROUTING_OPERATION_*`) | `int32 routing_time_s`, `bool success`, `string error_message`, `ProductData[<=1] product_data` | none |

### Services

| Service | Request | Response |
|---------|---------|----------|
| `SpawnProduct` | `product_number`, `product_revision`, `serial_number`, `assembly_model_id` (UUID4 of the SDF product model), `float64[6] pose` ([x, y, z, roll, pitch, yaw] in m/rad relative to the carrier model's `carrier_link`) | `bool success`, `string error_message` |
| `RemoveProduct` | — | `bool success`, `string error_message` |

### Provided (only while active)

| Name | Type | Notes |
|------|------|-------|
| `~/workplace` | `station_workplace/WorkplaceState` | Latched (reliable, transient local, depth 1); published only on change |
| `~/wait_routing` | `station_workplace/action/WaitRouting` | Action; see [wait_routing](#wait_routing) |
| TF `carrier` → `estimated_product_origin` | `/tf_static` | Not published before the first successful injection; reset to identity on ejection / injection error; never removed |
| `~/spawn_product` | `station_workplace/srv/SpawnProduct` | Only if `simulated`; see [Simulation](#simulation) |
| `~/remove_product` | `station_workplace/srv/RemoveProduct` | Only if `simulated`; see [Simulation](#simulation) |
| `/<barcode_scanner_node_name>/simulated_label_data` | `station_workplace/msg/LabelData` | barcode, only if `simulated`; latched (reliable, transient local, depth 1) |

### Consumed

| Name | Type | Used by |
|------|------|---------|
| `/<product_carrier_node_name>/product_in_place` | `std_msgs/Bool` (latched) | both |
| `/<product_carrier_node_name>/inject_product`, `/eject_product` | `std_srvs/srv/Trigger` | both (`wait_routing`) |
| `/<rfid_scanner_node_name>/tag_detected` | `std_msgs/Bool` (latched) | rfid |
| `/<rfid_scanner_node_name>/read_data` | `rfid_scanner/srv/ReadData` | rfid |
| `/<barcode_scanner_node_name>/label_data` | `station_workplace/msg/LabelData` | barcode |
| `/<production_manager_node_name>/get_production_data` | `production_manager/srv/GetProductionData` | both |
| `/<object_model_repository_node_name>/get_models_info` | `repositories/srv/ObjectModelStorageGetModelsInfo` | both |
| `/<object_model_repository_node_name>/get_model` | `repositories/srv/ObjectModelStorageGetModel` | both |
| `/get_planning_scene`, `/apply_planning_scene` | `moveit_msgs/srv/GetPlanningScene`, `ApplyPlanningScene` | both |
| `/world/<simulation.world_name>/create/blocking`, `/remove/blocking` | `ros_gz_interfaces/srv/SpawnEntity`, `DeleteEntity` | both, only if `simulated` |
| TF `world` → `<simulation.station_link>` | `/tf`, `/tf_static` | both, only if `simulated` |

## Parameters

### Common

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `product_carrier_node_name` | string | `conveyor` | Product carrier node name |
| `production_manager_node_name` | string | `production_manager` | Production manager node name |
| `object_model_repository_node_name` | string | `object_model_storage` | Object model storage node name |
| `collision_allowed_link` | string | `conveyor_origin` | Link the carrier model and the safety object may collide with |
| `safety_object.x` / `.y` / `.z` | double | `0.25` / `0.4` / `0.3` | Safety box size [m] (> 0) |
| `service_call_timeout_sec` | double | `5.0` | Timeout for service availability and for the response [s]; also the TF lookup timeout of `spawn_product` |
| `simulated` | bool | `false` | Provide `spawn_product` / `remove_product` (and, in the launch files, start the Gazebo bridge) |
| `simulation.world_name` | string | `default` | Gazebo world name |
| `simulation.station_model_name` | string | `station` | Carrier xacro argument `station_model_name` |
| `simulation.station_link` | string | `carrier` | Carrier xacro argument `station_link`; TF frame the carrier is spawned at |
| `simulation.carrier_model_name` | string | `carrier_model` | Name of the spawned carrier; carrier `model_name` and product `carrier_model_name` xacro argument |
| `simulation.carrier_link` | string | `carrier_link` | Carrier link the product is placed relative to; product xacro argument `carrier_link` |
| `simulation.product_model_name` | string | `product_model` | Name of the spawned product; product xacro argument `model_name` |
| `simulation.penetration_tolerance` | double | `0.001` | Product/carrier penetration depth [m] still treated as touching (>= 0) |

`simulation.*` can be given as launch arguments, `-p` overrides or YAML (dotted
keys or a nested `simulation:` map).

### `workplace_rfid_scanner`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rfid_scanner_node_name` | string | `simatic_rf240r_ascii` | RFID scanner node name |
| `tag_detected_timeout_sec` | double | `5.0` | Maximum wait for `tag_detected=True` after product arrival [s] |
| `tag_registers.<field>.start_address` | int | `0` / `32` / `64` | First tag address of `<field>` |
| `tag_registers.<field>.length` | int | `32` | Byte length of `<field>` (1..248) |

### `workplace_barcode_scanner`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `barcode_scanner_node_name` | string | `barcode_scanner` | Barcode scanner node name |
| `label_patterns.<field>` | string | `''` (**required**) | Regex with exactly one capture group extracting `<field>` from the label |

`<field>` is one of `product_number`, `product_revision`, `serial_number`.
Configure fails if any parameter is invalid (e.g. an empty pattern or one
without exactly one capture group).

## Behaviour

`product_in_place=True` starts an **injection**, `False` an **ejection**. Only
one process runs at a time; a value equal to the last handled one is ignored,
except the first value after each activation. `False` cancels a running
injection.

Injection:

1. Attach the safety box (`safety_object_model`) to `carrier`.
2. Read the product data (RFID tag fields, or the first **valid** label stamped
   after the arrival — barcode has no timeout; a label that cannot be decoded
   or does not match the `label_patterns` is logged and skipped, and the node
   keeps waiting for the next label until a valid one arrives or the product leaves).
3. `get_production_data` → `assembly_model_id`.
4. `get_models_info(model_id=assembly_model_id)`: exactly one `stl` model whose
   `metadata_json` contains `carrier_model_id` (UUID4) and
   `estimated_product_origin_pose` (`[x, y, z, roll, pitch, yaw]`, m/rad, relative to `carrier`).
5. `get_model` for the product, then `get_models_info` + `get_model` for the carrier (`stl`).
6. In one planning-scene update: remove the safety box, attach
   `carrier_object_model` (identity pose) and `product_object_model` (at the
   estimated pose; allowed to collide with the carrier model).
7. Publish the TF frame and `WorkplaceState{in_place: true, product_data: [...]}`.

Any failure in steps 2–7 is an **injection error**: product and carrier models
are removed, the safety box stays attached, the TF frame is reset to identity
and `in_place: true` with `UNKNOWN` product data is published.

Ejection removes all three objects, resets the TF frame and publishes
`WorkplaceState{in_place: false, product_data: []}`.

A failed ejection, or a failure to attach the safety box, is a **system
failure**: the node deactivates and cleans itself up (ends in *unconfigured*).

## wait_routing

`~/wait_routing` asks the product carrier to route a product into or out of the
workplace and waits until the workplace has settled. It exists only while the
node is active and handles one goal at a time; a second goal, or a goal with a
`routing_operation` other than `injection` / `ejection`, is rejected.

1. Mark the start time (node clock, respects `use_sim_time`).
2. If the workplace is already settled in the requested state (a product is in
   place for `injection`, none for `ejection`), finish immediately with the
   current state without calling the carrier.
3. Otherwise call `/<product_carrier_node_name>/inject_product` (or
   `eject_product`), bounded by `service_call_timeout_sec`; an unavailable
   service, a timeout or `success: false` fails the goal.
4. Wait, without a timeout, until the next injection (or ejection) process
   finishes, i.e. until the node publishes the resulting `~/workplace` state.
   The wait also finishes when that state equals the previous one and so is
   not re-published.

| Outcome | Goal status | `success` | `error_message` | `product_data` |
|---------|-------------|-----------|-----------------|----------------|
| Injection, product identified | SUCCEEDED | true | `''` | `[product]` |
| Injection, product `UNKNOWN` (injection error) | ABORTED | false | `Unknown product in station workplace` | `[UNKNOWN]` |
| Ejection finished | SUCCEEDED | true | `''` | `[]` |
| Carrier service failure | ABORTED | false | service error | `[]` |
| Client cancel | CANCELED | false | `wait_routing cancelled by the client` | `[]` |
| Node deactivated (also self-transition on a system failure) | ABORTED | false | `wait_routing aborted: node deactivated` | `[]` |

`routing_time_s` (whole seconds, rounded down) is reported in every case.
Cancelling only stops the wait; the carrier operation and the workplace
process are not affected. An injection that is cancelled by the product
leaving does not complete an `injection` goal; it keeps waiting until the
next injection finishes, the client cancels or the node deactivates.

```bash
ros2 action send_goal /workplace_rfid_scanner/wait_routing \
  station_workplace/action/WaitRouting "{routing_operation: injection}"
```

## Simulation

With `simulated:=true` the node provides `~/spawn_product` and `~/remove_product`.
They are independent of the workplace process: they neither read nor change
the workplace state or `product_in_place`. Both run serialized.

### `spawn_product`

1. Fails if a product spawned by this node is still recorded (call `remove_product` first).
2. Validates the request: non-empty ASCII product fields, UUID4 `assembly_model_id`,
   finite `pose`. RFID: each field must fit its `tag_registers.<field>.length`.
3. `get_models_info(assembly_model_id)`: exactly one `sdf` model whose `metadata_json`
   contains a UUID4 `carrier_model_id`; `get_model` returns its folder.
4. `get_models_info(carrier_model_id)`: exactly one `sdf` model; `get_model` returns its folder.
5. Expands `<folder>/model.sdf.xacro` of both models with the xacro arguments:
   - carrier: `model_name`, `station_model_name`, `station_link`; RFID additionally
     `tag_data` (hex string of the tag memory: each field NUL padded at its
     `tag_registers.<field>.start_address`, `length`);
   - product: `model_name`, `carrier_model_name`, `carrier_link`.
6. Checks the collision geometry (see below) of the product placed at
   `carrier_link ⊕ pose` against the carrier; penetration deeper than
   `simulation.penetration_tolerance` fails the call before anything is spawned.
7. Spawns the carrier at the TF pose of `world` → `simulation.station_link`, then
   the product at `carrier pose ⊕ carrier_link ⊕ pose`. The detachable joints in
   the models attach on spawn.
8. Barcode: publishes the label `<GS>1P<product_number><GS>21P<product_revision><GS>S<serial_number><RS><EOT>`
   (`GS`=0x1D, `RS`=0x1E, `EOT`=0x04) once on `simulated_label_data`.

If a step after the first spawn fails, the spawned models are removed (product
first) before the failure is returned.

### `remove_product`

Removes the recorded models, product first. Returns success immediately when
nothing is recorded. A failed removal returns `success=false`; models not
removed stay recorded so a later call retries them. The record survives
deactivate/activate (the models stay in the world when the node deactivates),
but not a process restart.

### SDF model contract

- The storage entry is a folder named by the model ID containing `model.sdf.xacro`
  with exactly one top-level `<model>` (no nested `<model>`/`<include>`).
- The folder's parent (`download_path`) must be on `GZ_SIM_RESOURCE_PATH` (set externally).
- Every `<uri>` must have a scheme; relative URIs are rejected because Gazebo
  cannot resolve them for an SDF spawned from a string. Collision meshes must use
  `model://<model_id>/<path>` (STL, OBJ, DAE; the DAE `<unit>` is applied).
- Supported collision shapes: `box`, `cylinder`, `sphere`, `mesh` (with `<scale>`).
  Other shapes, `<submesh>`, a model without collision geometry, poses in degrees
  or quaternions, and link/collision poses `relative_to` other frames fail the call.
- The carrier model has the link `simulation.carrier_link`; the product metadata
  contains `carrier_model_id`.

### Intersection check

Pairs of primitives use FCL's exact penetration depth. For pairs involving a
mesh, a colliding pair is accepted only if moving the product by the tolerance
along one of the 26 axis/diagonal directions of the carrier or product frame
separates it. A real penetration deeper than the tolerance is never accepted,
but a zero-clearance fit touching opposite or several non-axis-aligned faces is
rejected. A mesh completely inside another geometry without crossing its
surface is not detected.

```bash
ros2 launch station_workplace workplace_rfid_scanner.launch.py simulated:=true simulation.world_name:=default
ros2 service call /workplace_rfid_scanner/spawn_product station_workplace/srv/SpawnProduct \
  "{product_number: PN-1, product_revision: A, serial_number: SN-1, \
    assembly_model_id: 7c9e6679-7425-40de-944b-e07fc1f90ae7, pose: [0, 0, 0, 0, 0, 0]}"
ros2 service call /workplace_rfid_scanner/remove_product station_workplace/srv/RemoveProduct
```

## Lifecycle

The nodes start *unconfigured*; transitions are driven externally.

- **configure** validates the parameters (no ROS interfaces are created).
- **activate** creates all interfaces and starts reacting to `product_in_place`
  (and, if `simulated`, provides `spawn_product` / `remove_product`).
- **deactivate** abandons a running process, aborts a running `wait_routing`
  goal (the client still receives its result) and destroys the interfaces;
  planning-scene objects, the TF frame and spawned simulation models stay. The last published workplace
  state is remembered, so an identical state is not re-published after re-activation.
- **cleanup** / **shutdown** drop the configuration and the remembered state.

```bash
ros2 launch station_workplace workplace_rfid_scanner.launch.py
ros2 lifecycle set /workplace_rfid_scanner configure
ros2 lifecycle set /workplace_rfid_scanner activate

ros2 launch station_workplace workplace_barcode_scanner.launch.py \
  'label_patterns.product_number:=PN:(\S+)' \
  'label_patterns.product_revision:=REV:(\S+)' \
  'label_patterns.serial_number:=SN:(\S+)'
```

Every parameter is a launch argument of the same name (plus `node_name` and
`use_sim_time`), e.g. `safety_object.x:=0.3` or
`tag_registers.serial_number.length:=16`. With `simulated:=true` the launch
files also start a `ros_gz_bridge` (`<node_name>_gz_bridge`) for the world's
`create/blocking` and `remove/blocking` services.
