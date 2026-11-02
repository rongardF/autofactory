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
├── enums/{station_cache,object_model_storage}/
├── interfaces/{station_cache,object_model_storage}/
├── models/{station_cache,object_model_storage}/
├── services/{station_cache,object_model_storage}/
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
| Service | `~/get_models_info` | `ObjectModelStorageGetModelsInfo` | Models carrying **all** requested `filter_keys` pairs; empty filter returns all. |
| Service | `~/get_model` | `ObjectModelStorageGetModel` | Returns the absolute local path of one model. |
| Action | `~/get_models` | `ObjectModelStorageGetModels` | Processes `model_ids` in order, feedback (`model_id`, `local_path`) per model; fail-fast; cancelable between models. |
| Topic | `~/cached_models` | `ObjectModelStorageCachedModels` | Latched (transient local, depth 1); published on activate and after successful `get_model`/`get_models`. |

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
```

Drive the lifecycle with `ros2 lifecycle set /<node> configure` then
`... activate`.
