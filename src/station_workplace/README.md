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

### Provided (only while active)

| Name | Type | Notes |
|------|------|-------|
| `~/workplace` | `station_workplace/WorkplaceState` | Latched (reliable, transient local, depth 1); published only on change |
| TF `carrier` → `estimated_product_origin` | `/tf_static` | Not published before the first successful injection; reset to identity on ejection / injection error; never removed |

### Consumed

| Name | Type | Used by |
|------|------|---------|
| `/<product_carrier_node_name>/product_in_place` | `std_msgs/Bool` (latched) | both |
| `/<rfid_scanner_node_name>/tag_detected` | `std_msgs/Bool` (latched) | rfid |
| `/<rfid_scanner_node_name>/read_data` | `rfid_scanner/srv/ReadData` | rfid |
| `/<barcode_scanner_node_name>/label_data` | `station_workplace/msg/LabelData` | barcode |
| `/<production_manager_node_name>/get_production_data` | `production_manager/srv/GetProductionData` | both |
| `/<object_model_repository_node_name>/get_models_info` | `repositories/srv/ObjectModelStorageGetModelsInfo` | both |
| `/<object_model_repository_node_name>/get_model` | `repositories/srv/ObjectModelStorageGetModel` | both |
| `/get_planning_scene`, `/apply_planning_scene` | `moveit_msgs/srv/GetPlanningScene`, `ApplyPlanningScene` | both |

## Parameters

### Common

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `product_carrier_node_name` | string | `conveyor` | Product carrier node name |
| `production_manager_node_name` | string | `production_manager` | Production manager node name |
| `object_model_repository_node_name` | string | `object_model_storage` | Object model storage node name |
| `collision_allowed_link` | string | `conveyor_origin` | Link the carrier model and the safety object may collide with |
| `safety_object.x` / `.y` / `.z` | double | `0.25` / `0.4` / `0.3` | Safety box size [m] (> 0) |
| `service_call_timeout_sec` | double | `5.0` | Timeout for service availability and for the response [s] |

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

## Lifecycle

The nodes start *unconfigured*; transitions are driven externally.

- **configure** validates the parameters (no ROS interfaces are created).
- **activate** creates all interfaces and starts reacting to `product_in_place`.
- **deactivate** abandons a running process and destroys the interfaces;
  planning-scene objects and the TF frame stay. The last published workplace
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
`tag_registers.serial_number.length:=16`.
