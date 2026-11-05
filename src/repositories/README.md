# `repositories`

Repository-style ROS 2 nodes for storing and retrieving data on behalf of other
nodes in the system. Each repository node exposes a focused ROS 2 service surface
over a particular kind of data and a pluggable storage backend.

## Package layout

Python code is grouped per node inside each domain package; `utils/` is shared:

```
repositories/
├── station_cache.py               # station_cache node
├── object_model_storage.py        # object_model_storage node
├── operation_repository.py        # operation_repository node
├── enums/{station_cache,object_model_storage,operation_repository}/
├── interfaces/{station_cache,object_model_storage,operation_repository}/
├── models/{station_cache,object_model_storage,operation_repository}/
├── services/{station_cache,object_model_storage,operation_repository}/
└── utils/                         # shared helpers
```

ROS interface files stay flat in `msg/`, `srv/` and `action/`; every interface
carries its node's prefix (e.g. `StationCacheStoreString.srv`,
`ObjectModelStorageModelInfo.msg`).

## `station_cache` node

A typed key/value cache. Other nodes **store** and **fetch** typed values through
ROS 2 services; entries are scoped per `station_id` and may optionally expire.

- **Class:** `StationCache` (`rclpy.lifecycle.LifecycleNode`)
- **Executable:** `station_cache`
- **Backends:** `LocalCacheBackend` (in-memory, default) / `CloudCacheBackend`
  (REST — stubbed this iteration)

### Supported value types

`string`, `boolean`, `number` (float64), `string[]`, `bytes` (uint8[]),
`number[]` (float64[]).

### Service surface (node-relative, resolve to `/station_cache/<name>`)

| Store | Fetch |
|-------|-------|
| `~/store_string` | `~/fetch_string` |
| `~/store_boolean` | `~/fetch_boolean` |
| `~/store_number` | `~/fetch_number` |
| `~/store_string_array` | `~/fetch_string_array` |
| `~/store_bytes` | `~/fetch_bytes` |
| `~/store_number_array` | `~/fetch_number_array` |

Plus one type-agnostic `~/delete`.

### Parameters

| Parameter | Type | Default | Notes |
|-----------|------|---------|-------|
| `local_cache` | bool | `true` | `true` → local in-memory; `false` → cloud (stubbed). |
| `station_id` | string | `''` | **Required.** Must be a UUID4 string. |
| `cache_service_url` | string | `''` | Required only when `local_cache = false`. |
| `expiry_sweep_interval_sec` | double | `1.0` | Must be `> 0`. |
| `request_timeout_sec` | double | `5.0` | Provisional; unused until cloud mode. |

Parameters are read and frozen on `configure`. Services exist only while the node
is `active`; local cache contents survive deactivate/activate cycling and are
cleared only on `cleanup`/`shutdown`.

## `object_model_storage` node

Looks up 3D object model metadata and makes model files (STL/DAE/OBJ files or
Gazebo SDF model folders) available locally, returning their absolute paths.

- **Class:** `ObjectModelStorage` (`rclpy.lifecycle.LifecycleNode`)
- **Executable:** `object_model_storage`
- **Backends:** `LocalModelStorageBackend` (`local_storage.json`, no network) /
  `CloudModelStorageBackend` (REST — HTTP transport stubbed this iteration)

Model metadata is never cached — every request reads the source afresh. Model
files are cached in `download_path`, each named by its UUID4 `model_id` (no
extension; cloud downloads are ZIP archives extracted into `<download_path>/<model_id>/`).
Existing cached entries are reused, never re-downloaded.

### ROS surface (node-relative, resolve to `/object_model_storage/<name>`)

| Kind | Name | Type | Notes |
|------|------|------|-------|
| Service | `~/get_models_info` | `ObjectModelStorageGetModelsInfo` | One model by `model_id`, or models carrying **all** requested `filter_keys` pairs; empty filter returns all. |
| Service | `~/get_model` | `ObjectModelStorageGetModel` | Returns the absolute local path of one model. |
| Action | `~/get_models` | `ObjectModelStorageGetModels` | Processes `model_ids` in order, feedback (`model_id`, `local_path`) per model; fail-fast; cancelable between models. |
| Topic | `~/cached_models` | `ObjectModelStorageCachedModels` | Latched (transient local, depth 1); published on activate and after successful `get_model`/`get_models`. |

`~/get_models_info` lookup by id: a non-empty `model_id` (UUID4) takes precedence
and `filter_keys` are ignored. An unknown `model_id` returns `success=true` with an
empty `models` list; a malformed (non-UUID4) `model_id` returns `success=false`.

All responses carry `success` + `error_message`; failures never raise across the
ROS boundary.

### Parameters

| Parameter | Type | Default | Notes |
|-----------|------|---------|-------|
| `local_storage` | bool | `true` | `true` → `local_storage.json`; `false` → cloud REST (stubbed). |
| `download_path` | string | `/tmp/station/object_models/` | Must be absolute; created on `configure` if missing. |
| `station_id` | string | `''` | **Required.** Must be a UUID4 string. |
| `storage_service_url` | string | `''` | Required only when `local_storage = false`. |
| `request_timeout_sec` | double | `5.0` | Must be `> 0`; unused until cloud mode is implemented. |

### Local mode

Place `local_storage.json` and the model entries in `download_path`:

```json
[
  {
    "model_id": "11111111-1111-4111-8111-111111111111",
    "model_type": "stl",
    "filter_keys": [{"key": "kind", "value": "bracket"}],
    "metadata_json": {"mass_kg": 0.4}
  }
]
```

`model_type` is one of `sdf`, `stl`, `dae`, `obj`. `metadata_json` may be a JSON
string, a JSON object, or omitted/`null`. Every listed model must exist in
`download_path` as a file or folder named by its `model_id`; local mode never
downloads.

### Cloud mode (contract; HTTP not implemented yet)

- `GET {storage_service_url}/{station_id}/object_model_storage/get_models_info` →
  JSON array of model info objects.
- `GET {storage_service_url}/{station_id}/object_model_storage/download?model_id=<uuid4>` →
  ZIP archive of the model.

Until the service exists, cloud-mode calls return `success=false` with
`cloud mode not implemented (GET <url>)`.

## `operation_repository` node

Returns one validated assembly operation by `assembly_id` and `operation_id`.
The node is the single owner of operation DTOs and dispensing-pattern ROS
messages; `pattern_executors` consumes those shared messages.

- **Class:** `OperationRepository` (`rclpy.lifecycle.LifecycleNode`)
- **Executable:** `operation_repository`
- **Service:** `~/get_operation`
  (`repositories/srv/OperationRepositoryGetOperation`)
- **Backends:** `LocalOperationBackend` (assembly JSON, loaded on configure) /
  `CloudOperationBackend` (REST contract; HTTP transport is stubbed)

The service returns exactly one `OperationRepositoryOperation` on success and
an empty `operation` array on failure. Failures are logged and returned as
`success=false` with an explanatory `error_message`.

### Parameters

| Parameter | Type | Default | Notes |
|-----------|------|---------|-------|
| `local_repository` | bool | `true` | Local assembly file when true; cloud REST backend when false. |
| `assembly_file_path` | string | `/tmp/station/mock_assembly_file.json` | Must be absolute; read and fully validated on configure. |
| `station_id` | string | `''` | Required UUID4 in cloud mode only. |
| `operation_repository_url` | string | `''` | Required in cloud mode only. |
| `product_origin_frame_id` | string | `product_origin` | Must not be empty; assigned to all converted poses. |

### Local mode

The assembly root requires `schema_version: "1.0"` and a UUID4 `id`.
Every operation is parsed and validated when the node is configured; a missing
or unreadable file, unsupported schema, malformed operation, or invalid UUID
fails configure. Requests must provide the assembly's root id and an operation
id present in the cached assembly.

### Cloud mode (contract; HTTP not implemented yet)

`GET {operation_repository_url}/{station_id}/operation_repository/operation_data`
with `assembly_id`, `operation_id`, and `schema_version=1.0` query parameters
returns one operation object. Until the service exists, the backend makes no
HTTP request and returns `success=false` with the requested URL in the error.
Cloud configuration requires a UUID4 `station_id` and a non-empty URL.

### JSON-to-message conversion

The operation DTO validates its root schema version and required fields.
Pose arrays are interpreted as XYZ meters plus fixed-axis roll/pitch/yaw radians
and converted to `PoseStamped` in `product_origin_frame_id`. JSON null numeric
movement settings become `-1.0`, a null `circ_type` becomes an empty string,
and a null circular point becomes an empty bounded array. Operation, pattern,
and action arguments use `OperationRepositoryArg[]`; each argument's
`argument_type` selects its one active typed value field, while all other value
fields remain at their message defaults. Operation argument values are strings
or other supported JSON types; pattern and action arguments retain their JSON
value types. Empty argument objects and `null` are represented as `null` in the
DTO's JSON form and as empty arrays in ROS messages. JSON value arrays must be
non-empty and homogeneous so their element type can be inferred. The shared
`SCHEMA_VERSION` constant is `1.0`. Dispensing patterns still require the
`flowrate_m3_s` float argument, and wait actions require their `wait_time_s`
float argument.

## Build

```bash
colcon build --packages-select repositories --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run repositories station_cache --ros-args \
  -p station_id:=<uuid4> -p local_cache:=true

ros2 run repositories object_model_storage --ros-args \
  -p station_id:=<uuid4> -p download_path:=/tmp/station/object_models/

ros2 run repositories operation_repository
```

Drive the lifecycle with `ros2 lifecycle set /<node> configure` then
`... activate`. For the local example, copy
`mock_product_assembly.json` to `/tmp/station/mock_assembly_file.json`, then
call `/operation_repository/get_operation` with the assembly and operation
UUID4 values.
