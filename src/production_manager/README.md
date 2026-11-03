# Production Manager

The `production_manager` ROS 2 lifecycle node selects the current assembly operation for a product and reports its outcome. It provides a local JSON-backed mode for development and a cloud backend placeholder for later REST integration.

## Modes

- **Local manager** (`local_manager:=true`): reads and validates the assembly JSON file once during configuration. Use `~/set_operation` to assign the operation and serial number that the next `~/get_production_data` request should return.
- **Cloud backend** (`local_manager:=false`): validates the service URL and exposes the same production-data and reporting services. Both return `success=false` and `error_message="not implemented"`; no HTTP requests are made.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `local_manager` | bool | `true` | Select local JSON or cloud backend stub. |
| `assembly_file_path` | string | `/tmp/station/mock_assembly_file.json` | Absolute path to the local assembly JSON. Required to exist in local mode. |
| `station_id` | string | `''` | Station UUID4. Required in both modes. |
| `manager_service_url` | string | `''` | REST service base URL; must be HTTP(S) in cloud mode. |
| `use_sim_time` | bool | `false` | ROS clock configuration. |

## Services

All service names are relative to the node namespace.

### `~/get_production_data` — `production_manager/srv/GetProductionData`

Request: `product_number`, `product_revision`, `serial_number`.

Response: `assembly_id`, `operation_id`, `assembly_model_id`, `success`, `error_message`. On failure, the three IDs are empty.

### `~/report_production` — `production_manager/srv/ReportProduction`

Request: UUID4 `assembly_id`, UUID4 `operation_id`, and the operation outcome `success`.

Response: whether the report was accepted and an `error_message`.

### `~/set_operation` — `production_manager/srv/SetOperation`

Available only in local mode. Request: UUID4 `operation_id`, `serial_number`, and UUID4 `station_id`. Response: `success` and `error_message`.

## Lifecycle

The launch file starts the node in the unconfigured state. An external lifecycle manager must:

1. Configure the node. Parameters are validated, and local mode loads and validates the assembly JSON.
2. Activate the node. The production services are created; local mode also creates `~/set_operation`.
3. Deactivate the node to remove the services. Cleanup discards the backend and configuration; configure can then be run again.

Example:

```bash
ros2 lifecycle set /production_manager configure
ros2 lifecycle set /production_manager activate
```

## Assembly JSON

The file is loaded as a JSON object and is not parsed into a fixed model. Its root `schema_version` controls interpretation of root-level and operation-level fields. Version `1.0` is currently supported. The root must contain UUID4 `id`, string `product_number` and `product_revision`, and a list of operation objects. Each operation must contain UUID4 `id` and `start_assembly_model_id`. Other fields are ignored.

`start_assembly_model_id` identifies the product model before the selected operation. The operation's internal `patterns` are not read.

## Local workflow

Prepare a local copy of the example assembly file, then launch with a station UUID4:

```bash
mkdir -p /tmp/station
cp src/production_manager/example_assembly_file.json /tmp/station/mock_assembly_file.json
ros2 launch production_manager production_manager_launch.py \
  station_id:=<station-uuid4>
```

After configuring and activating the node, call the services in order:

```bash
ros2 service call /production_manager/set_operation \
  production_manager/srv/SetOperation \
  "{operation_id: 'dab06081-0e24-4583-9f33-06d3c26b69f7', serial_number: 'SERIAL-001', station_id: '<station-uuid4>'}"

ros2 service call /production_manager/get_production_data \
  production_manager/srv/GetProductionData \
  "{product_number: 'ABC123/1', product_revision: '1.2', serial_number: 'SERIAL-001'}"

ros2 service call /production_manager/report_production \
  production_manager/srv/ReportProduction \
  "{assembly_id: '6384f49c-a0d5-4d8d-bdcf-a45c6f02d20d', operation_id: 'dab06081-0e24-4583-9f33-06d3c26b69f7', success: true}"
```

Replace `<station-uuid4>` with the same UUID4 passed to the launch file.

## Testbench notebook

The manual client notebook is `tests/testbench/testbench.ipynb`. Run it from a Jupyter kernel using the `/opt/venv` environment after sourcing ROS 2 Jazzy and the workspace overlay. Start the node in local mode and configure and activate it first. Fill in the notebook's placeholder values before calling the service cells. The notebook does not perform lifecycle transitions.

## Launch

```bash
ros2 launch production_manager production_manager_launch.py \
  local_manager:=true \
  assembly_file_path:=/tmp/station/mock_assembly_file.json \
  station_id:=<station-uuid4>
```

For the cloud stub, set `local_manager:=false` and supply `manager_service_url:=https://...`.
