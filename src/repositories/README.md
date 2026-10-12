# `repositories`

Repository-style ROS 2 nodes for storing and retrieving data on behalf of other
nodes in the system. Each repository node exposes a focused ROS 2 service surface
over a particular kind of data and a pluggable storage backend.

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

## Build

```bash
colcon build --packages-select repositories --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run repositories station_cache --ros-args \
  -p station_id:=<uuid4> -p local_cache:=true
```

Drive the lifecycle with `ros2 lifecycle set /station_cache configure` then
`... activate`.
