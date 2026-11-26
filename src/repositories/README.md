# `repositories`

Repository-style ROS 2 nodes for storing and retrieving data on behalf of other
nodes in the system. Each repository node exposes a focused ROS 2 service surface
over a particular kind of data and a pluggable storage backend.

## Package layout

Python code is grouped per node inside each domain package; `utils/` and the
`common/` sub-packages are shared:

```
repositories/
├── station_cache.py               # station_cache node
├── object_model_storage.py        # object_model_storage node
├── operation_repository.py        # operation_repository node
├── image_repository.py            # image_repository node
├── material_repository.py         # material_repository node
├── blob_repository.py             # blob_repository node
├── enums/{station_cache,object_model_storage,operation_repository,image_repository,material_repository}/
├── exceptions/{image_repository,material_repository,blob_repository}/
├── interfaces/{station_cache,object_model_storage,operation_repository,image_repository,material_repository,blob_repository}/
├── models/{common,station_cache,object_model_storage,operation_repository,image_repository,material_repository,blob_repository}/
├── services/{common,station_cache,object_model_storage,operation_repository,image_repository,material_repository,blob_repository}/
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

### Message-to-DTO conversion

`OperationDTO.from_msg(OperationRepositoryOperation)` is the inverse of `to_msg`
(also available on the nested pattern, movement, action, material and end-tool
DTOs). Message values of `-1.0` become `null`, an empty `circ_type` becomes
`null`, and quaternions are converted back to roll/pitch/yaw with
`repositories.utils.pose_converter.pose_stamped_to_pose6`. At a pitch of
+/- pi/2 the roll is reported as `0` and the rotation is carried by the yaw,
so compare round trips by orientation, not by RPY values. The frame id of the
poses is dropped; `to_msg(frame_id)` restamps them. `dispensing_patterns` is
filled only for `dispensing` operations.

## `material_repository` node

A read-only repository of material data, referenced by **product number and
revision**, that endtools use to learn about the material they process (for
example the density and color of the putty a dispensing endtool dispenses).
Nothing in this node writes material data.

- **Class:** `MaterialRepository` (`rclpy.lifecycle.LifecycleNode`)
- **Executable:** `material_repository`
- **Service:** `~/get_material_data`
  (`repositories/srv/MaterialRepositoryGetMaterialData`), created on `activate`
  and destroyed on `deactivate`
- **Backends:** `LocalMaterialBackend` (JSON file, parsed on `configure`) /
  `CloudMaterialBackend` (stub: no HTTP calls are made)

The request carries `product_number` and `product_revision`; the lookup is a
**case-insensitive exact match** on that pair (stored and returned values keep the
casing of the file). The response holds a `MaterialRepositoryMaterialData`
(`product_number`, `product_revision`, `material_type`, `is_serialized`,
`payload_json`), `success` and `error_message`. On failure `material_data` is
empty and `error_message` is `material not found` (no match) or
`cloud mode not implemented` (cloud mode); failures are logged and never raise
across the ROS boundary.

### Parameters

| Parameter | Type | Default | Notes |
|-----------|------|---------|-------|
| `station_id` | string | `''` | **Required.** Must be a UUID4 string. |
| `local_storage` | bool | `true` | `true` → local materials file; `false` → cloud (stubbed). |
| `material_service_url` | string | `''` | Required only when `local_storage = false`. |
| `request_timeout_sec` | double | `5.0` | Must be `> 0`; unused until cloud mode is implemented. |
| `materials_file_path` | string | `/tmp/station/materials.json` | Local mode only; must be absolute. |

Parameters are read and frozen on `configure`; an invalid value fails the
transition. In cloud mode all lifecycle transitions succeed and every lookup
answers `cloud mode not implemented`.

### Lifecycle

| Transition | Behaviour |
|------------|-----------|
| `configure` | Builds the frozen config, creates the internal material service (not yet initialized) and, in local mode, parses and validates the materials file into an in-memory mapping `(number, revision) → material`. A missing, unreadable, invalid-JSON or invalid file fails the transition. |
| `activate` | Creates `~/get_material_data` and initializes the internal service. |
| `deactivate` | De-initializes the internal service and destroys `~/get_material_data`. |
| `cleanup` / `shutdown` | Remove the config, the cache and the internal service. |

### Local mode

`materials_file_path` points to a JSON file with a top-level list of materials,
see [example_material_file.json](./example_material_file.json):

```json
[
  {
    "product_number": "material-123",
    "product_revision": "a1",
    "material_type": "fluid",
    "is_serialized": false,
    "payload_json": {
      "density_kg_m3": 3150.0,
      "viscosity_pa_s": 20.0,
      "color_rgba": [1.0, 0.631, 0.0, 1.0]
    }
  }
]
```

Every entry holds the common fields (`product_number`, `product_revision`,
`material_type`, `is_serialized`) plus a nested `payload_json` **object**, which
the node serializes to a string for the message. Loading is strict: any invalid
entry fails the load. Invalid means a missing common field, an unknown
`material_type` (`fluid` or `screw`), a payload that does not validate against
its type model (`screw` payloads only have to be JSON objects), or a duplicate
(`product_number`, `product_revision`) pair, compared case-insensitively.

### Material models

The payload models live in this package and are imported by the consuming
nodes: `repositories.models.material_repository.FluidMaterial` and `ScrewMaterial`
(both based on `Material`). They are frozen Pydantic v2 models without default
values, allow extra fields (preserved when converting back to a message) and
provide `to_msg()` / `from_msg()` for the full `MaterialRepositoryMaterialData`
message:

```python
from repositories.models.material_repository import FluidMaterial

material = FluidMaterial.from_msg(response.material_data)  # raises ValueError if invalid
material.density_kg_m3
```

`FluidMaterial` carries `density_kg_m3` (> 0), `viscosity_pa_s` (> 0) and
`color_rgba` (exactly four values, each 0.0–1.0). `ScrewMaterial` only names the
type; no fields or behaviour are implemented yet. A consumer that cannot parse
`payload_json` into its model must treat that as a failure (the dispensing
endtool fails its configuration).

### Cloud mode (stub)

No HTTP calls are made; the service contract will be defined once a cloud
material service exists.

## `image_repository` node

Lets other nodes **upload** images to and **fetch** images from a storage. Every
image is kept on the local filesystem; in remote mode the remote image service is
the source of truth and the local directory is a cache.

- **Class:** `ImageRepository` (`rclpy.lifecycle.LifecycleNode`)
- **Executable:** `image_repository`
- **Backends:** local filesystem only (default) / `RemoteImageBackend` (REST —
  stubbed this iteration; every remote call returns `remote mode not implemented`)

### Service surface (node-relative, resolve to `/image_repository/<name>`)

| Service | Type | Notes |
|---------|------|-------|
| `~/upload_image` | `ImageRepositoryUploadImage` | Stores one `sensor_msgs/Image` with a name and optional JSON metadata; returns a UUID4 `image_id`. |
| `~/upload_images` | `ImageRepositoryUploadImages` | Same for several images; all-or-nothing with rollback. |
| `~/fetch_image` | `ImageRepositoryFetchImage` | Returns the image, name and metadata for one `image_id`. |
| `~/fetch_images` | `ImageRepositoryFetchImages` | Same for several ids; all-or-nothing. |

All responses carry `success` + `error_message`; failures never raise across the
ROS boundary. Metadata must be valid JSON (or empty) and is returned in compact
form. Supported encodings: `mono8`, `mono16`, `rgb8`, `bgr8`, `rgba8`, `bgra8`,
`rgb16`, `bgr16`, `rgba16`, `bgra16`.

### Parameters

| Parameter | Type | Default | Notes |
|-----------|------|---------|-------|
| `local_storage` | bool | `true` | `true` → local only; `false` → remote service plus local cache (stubbed). |
| `local_storage_path` | string | `/tmp/station/images` | Must be absolute; created on `configure` if missing. |
| `station_id` | string | `''` | **Required** in both modes. Must be a UUID4 string. |
| `image_service_url` | string | `''` | Required only when `local_storage = false`. |
| `request_timeout_sec` | double | `5.0` | Must be `> 0`; unused until remote mode is implemented. |
| `cache_size_mb` | int | `50` | Must be `> 10`. Eviction starts when the directory exceeds `cache_size_mb - 10` MB. |
| `cache_check_interval_sec` | double | `60.0` | Must be `> 0`. Period of the size check while `active`. |

### Storage layout and cache eviction

`local_storage_path` holds `<image_id>` (PNG, no extension) and
`<image_id>_metadata.json` per image. **The directory must be dedicated to this
node:** on `configure`, orphaned files, leftover temp files and any other regular
file are deleted (subdirectories and symlinks are left alone).

When the directory would exceed `cache_size_mb - 10` MB (before a write, after a
store, on `configure` and periodically while active), the oldest image/metadata
pairs are deleted together until it fits. The images of the current request and
the most recently stored image are never evicted, and uploads never fail for lack
of cache space. In remote mode eviction only drops the local copy; in local mode
it permanently deletes the image.

`LocalImageStore` is built on the shared `ManagedFileStore` core
(`services/common/`), which `blob_repository` also uses.

```bash
ros2 run repositories image_repository --ros-args \
  -p station_id:=<uuid4> -p local_storage_path:=/tmp/station/images
```

## `blob_repository` node

Stores and retrieves **arbitrary files** as opaque byte streams. Clients upload
a file by filesystem path and get a UUID4 `blob_id`; they fetch by id and get a
local path. File content never crosses ROS messages: the node streams it between
the filesystem, its storage directory and (in cloud mode) the cloud service, so
clients and the node must share the filesystems behind `allowed_path_roots`.
Content is never interpreted, converted or executed.

- **Class:** `BlobRepository` (`rclpy.lifecycle.LifecycleNode`)
- **Executable:** `blob_repository`
- **Backends:** `LocalBlobBackend` (local filesystem is the source of truth,
  default) / `RemoteBlobBackend` (station-scoped HTTPS REST service is the source
  of truth; the local directory is a cache)
- **Limits:** max blob size 1 GiB, storage/cache size 3 GiB (defaults)

### ROS surface (node-relative, resolve to `/blob_repository/<name>`)

| Name | Type | Notes |
|------|------|-------|
| `~/upload_blob` | `BlobRepositoryUploadBlob` (srv) | Upload `source_path`; optional `blob_id` (UUID4 upsert), `blob_name` (defaults to the basename; metadata only), `media_type` (inferred from the file name, else `application/octet-stream`), `metadata` (JSON). Returns id, name, media type, `size_bytes`, `sha256`. |
| `~/fetch_blob` | `BlobRepositoryFetchBlob` (srv) | Make one blob local. Empty `destination_path` returns the managed cache path; otherwise the content is copied to the destination. `download_fresh` (cloud mode only) re-downloads even on a cache hit. Returns `local_path` and the descriptor. |
| `~/get_blob_info` | `BlobRepositoryGetBlobInfo` (srv) | Descriptor only; no content transfer. |
| `~/delete_blob` | `BlobRepositoryDeleteBlob` (srv) | Delete the blob and its local copy; an absent blob succeeds, an invalid id fails. |
| `~/upload_blobs` | `BlobRepositoryUploadBlobs` (action) | Sequential, **all-or-nothing** batch upload with per-item feedback. Optional arrays (`blob_ids`, `blob_names`, `media_types`, `metadata`) are empty or the same length as `source_paths`. On failure or cancel everything written by the batch is rolled back (overwritten blobs are restored); ids whose cloud rollback failed are named in `error_message`. |
| `~/fetch_blobs` | `BlobRepositoryFetchBlobs` (action) | Sequential, **fail-fast** batch fetch with per-item feedback; `destination_paths` is empty or the same length as `blob_ids`. Items fetched before a failure/cancel stay available, but no paths are returned. |

Services and actions exist only while the node is `active`. Every response
carries `success` + `error_message`; failures are logged and returned, never
raised across the ROS boundary, and failed responses have empty/zero data
fields. Requests are validated before any storage work (empty batches,
mismatched arrays, invalid UUID4s, malformed JSON metadata or media types).
Cancellation is checked between batch items; single-file services cannot be
canceled.

### Path policy

- `source_path` must be absolute and resolve (after `..` and symlink
  resolution) to a readable regular file under one of `allowed_path_roots`.
- An explicit `destination_path` must be absolute, its parent directory must
  exist inside an allowed root, and an existing entry must be a regular file
  (it is **overwritten**); symlinks and other non-regular entries are rejected.
  The content is copied to a hidden temp file next to the destination, verified
  (size + SHA-256) and atomically renamed over it, so a failed transfer leaves an
  existing file unchanged.
- The node's own `local_storage_path` is never a valid source or destination,
  and `blob_name` never contributes to any path.

### Parameters

| Parameter | Type | Default | Notes |
|-----------|------|---------|-------|
| `local_storage` | bool | `true` | `true` → local disk is the source of truth; `false` → cloud service plus local cache. |
| `local_storage_path` | string | `/tmp/station/blobs` | Absolute; created on `configure`; **dedicated to this node**. |
| `station_id` | string | `''` | **Required** UUID4 in both modes. |
| `blob_service_url` | string | `''` | Required in cloud mode; must be an `https://` URL without credentials, query or fragment. |
| `request_timeout_sec` | double | `5.0` | `> 0`; connect and metadata request timeout. |
| `transfer_timeout_sec` | double | `600.0` | `> 0`; total time budget for streaming one file. |
| `cache_size_mb` | int | `3072` | `> 0` and `>= max_blob_size_mb`; strict limit of the storage/cache directory. |
| `max_blob_size_mb` | int | `1024` | `> 0`; larger files are rejected before transfer. |
| `cache_check_interval_sec` | double | `60.0` | `> 0`; period of the cache limit check while `active`. |
| `allowed_path_roots` | string[] | `[]` | **At least one** absolute, existing directory. |

Parameters are frozen into `BlobRepositoryConfigDTO` on `configure`; changes
apply only after `cleanup` + `configure`. `configure` fails on invalid values,
an unusable storage directory or a missing allowed root.

### Storage layout and cache policy

`local_storage_path` holds `<blob_id>` (raw content) and
`<blob_id>_metadata.json` (descriptor: id, name, media type, byte length,
SHA-256, `created_at`/`updated_at`, user metadata) per blob. Writes are staged
in hidden files and atomically renamed into place; a supplied `blob_id` replaces
content and descriptor together (`created_at` is kept). On `configure`, hidden
temp/staging/backup files, incomplete pairs, descriptors that do not match their
content size and unrelated regular files are deleted (subdirectories and
symlinks are left alone). `deactivate`, `cleanup` and `shutdown` never delete
valid blobs.

The directory never exceeds `cache_size_mb`: before each write, the oldest blobs
(content modification time, then id) are evicted until the new file fits; blobs
of the current request and blobs being read or written are never evicted. If the
space still cannot be made available, the operation fails. A timer enforces the
limit while active. **In local mode eviction permanently deletes blobs**
(intentional v1 behavior: size `cache_size_mb` for the data you must keep); in
cloud mode it only drops the cached copy.

### Cloud mode (REST contract)

With `{base}` = `{blob_service_url}/{station_id}/blob_repository/blobs`:

| Method and path | Behavior |
|---|---|
| `POST {base}` | Multipart upload; the server allocates and returns a UUID4. |
| `PUT {base}/{blob_id}` | Multipart upsert at the supplied UUID4. |
| `GET {base}/{blob_id}` | Stream the content. |
| `GET {base}/{blob_id}/info` | Descriptor JSON. |
| `DELETE {base}/{blob_id}` | Delete; `404` counts as deleted. |

Multipart bodies contain a `descriptor` part (JSON: `blob_name`, `media_type`,
`size_bytes`, `sha256`, `metadata`) followed by a `file` part with the content.
`POST`/`PUT` and `/info` respond with the stored descriptor JSON (those fields
plus `blob_id`, `created_at`, `updated_at`). `404` means not found, `400`
invalid, `413` too large; response bodies are never echoed into errors.
Transfers are streamed in 1 MiB chunks in both directions; downloads are verified
(size + SHA-256) before they become visible. Uploads go to the cloud first and
then to the cache. Before a supplied id is overwritten, its current content is
kept as a hidden local backup (taken from the cache, or downloaded) so a failed
batch can restore it.

Authentication: set the bearer token in the **`BLOB_REPOSITORY_AUTH_TOKEN`**
environment variable of the node process (via the deployment's secret
mechanism; never commit it). It is sent as `Authorization: Bearer <token>` and
never logged. If it is unset, `configure` logs a warning and requests are sent
without authentication.

```bash
ros2 run repositories blob_repository --ros-args \
  -p station_id:=<uuid4> -p "allowed_path_roots:=['/data/station']"
ros2 lifecycle set /blob_repository configure
ros2 lifecycle set /blob_repository activate
ros2 service call /blob_repository/upload_blob repositories/srv/BlobRepositoryUploadBlob \
  "{source_path: /data/station/program.json}"
ros2 service call /blob_repository/fetch_blob repositories/srv/BlobRepositoryFetchBlob \
  "{blob_id: <uuid4>, destination_path: /data/station/copy.json}"
```
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

ros2 run repositories material_repository --ros-args -p station_id:=<uuid4>

ros2 run repositories blob_repository --ros-args \
  -p station_id:=<uuid4> -p "allowed_path_roots:=['/data/station']"
```

Drive the lifecycle with `ros2 lifecycle set /<node> configure` then
`... activate`. For the local example, copy
`mock_product_assembly.json` to `/tmp/station/mock_assembly_file.json`, then
call `/operation_repository/get_operation` with the assembly and operation
UUID4 values.
