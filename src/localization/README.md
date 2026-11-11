# localization

Product localization. The package provides the lifecycle node `mock_localizer`, which
stands in for the vision based localizer: it exposes the same interface and performs the
real robot and camera work, but it does not calculate the product pose.

## Flow of `localize_product`

1. Move the camera TCP (`camera_frame_id`) to `initial_localization_pose` (in `base_frame_id`),
   wait `image_capture_settling_s`, capture an image.
2. Publish TF `localization_origin` (hardcoded estimate of the product centre in the mock).
3. Move to every pose of `calibration_poses_file` (relative to `localization_origin`), settle,
   capture an image. Send feedback `collection_done=true`; no movement is commanded afterwards.
4. Mocked calculation: wait 20 s, then publish `product_origin` (parent `carrier_frame_id`)
   as a copy of TF `carrier` -> `estimated_product_origin`, published by `station_workplace`.

`localization_origin` is published until the action ends; `product_origin` while `~/localized`
is true. Both are dynamic transforms on `/tf` (10 Hz): TF cannot remove a frame, so a frame
disappears from the listeners' buffers shortly after publishing stops.

## Interface

| Name | Type | Notes |
|---|---|---|
| `~/localize_product` | `localization/action/LocalizeProduct` | Goal: `assembly_model_id` (UUID4). Rejected if the id is invalid, a goal is running, or no product is in place. Cancellable. |
| `~/localized` | `std_msgs/Bool` | Latched. True after a successful localization until the workplace state changes (product leaves or differs), a new goal is accepted or the node is deactivated. |
| `/<station_workplace_node_name>/workplace` | `station_workplace/msg/WorkplaceState` | Subscribed. |
| `/<movement_controller_node_name>/execute_trajectory` | `movement_controller/action/ExecuteTrajectory` | Client; PTP moves, target poses transformed to `base_link`. |
| `/<camera_node_name>/get_image` | `cameras/action/GetImage` | Client; undistorted image taken after the request. |

Result: `localized_product_frame` (`product_origin`), `localization_time_s`, `success`,
`error_message`. Feedback: `collection_done`, `collection_time_s`.

The action fails if the product leaves or changes while it runs.

## Lifecycle

The node never changes its own state.

| Transition | Behaviour |
|---|---|
| configure | Validate parameters, load the poses file (fails if missing, invalid or empty), create the mock service. |
| activate | Initialize the service (TF, workplace subscription, action clients), create the action server and `~/localized` (published false). |
| deactivate | Cancel a running goal, tear down the service, destroy the interface. |
| cleanup | Drop the configuration and the service. |
| shutdown | Release everything, from any state. |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `station_workplace_node_name` | `station_workplace` | Node publishing `~/workplace`. |
| `movement_controller_node_name` | `movement_controller` | Node serving `execute_trajectory`. |
| `camera_node_name` | `cameras/basler_camera` | Camera node including its namespace. |
| `camera_frame_id` | `camera_tcp_calibrated` | Camera TCP, used as tool frame. |
| `base_frame_id` | `station` | Frame of the initial pose; parent of `localization_origin`. |
| `carrier_frame_id` | `carrier` | Parent of `product_origin`. |
| `initial_localization_pose` | `[1.0, 0.0, 1.46, 0.0, 1.5708, 0.0]` | `[x, y, z, roll, pitch, yaw]` (m, rad) in `base_frame_id`. Placeholder. |
| `calibration_poses_file` | `''` | Absolute path of the poses YAML (required). |
| `image_capture_settling_s` | `0.5` | Wait between reaching a pose and capturing. |
| `image_capture_timeout_s` | `5.0` | Time allowed for the camera to return an image. |

The launch file exposes every parameter as an argument of the same name
(`initial_localization_pose` as `x,y,z,roll,pitch,yaw`), plus `node_name` and `use_sim_time`.

### Poses file

```yaml
poses:
  - [x, y, z, roll, pitch, yaw]   # relative to localization_origin; m, rad
```

See `config/localization_poses.yaml` (placeholder values).

## Run

```bash
ros2 launch localization mock_localizer.launch.py
ros2 lifecycle set /mock_localizer configure
ros2 lifecycle set /mock_localizer activate
ros2 action send_goal --feedback /mock_localizer/localize_product \
  localization/action/LocalizeProduct "{assembly_model_id: 'edf9129e-6656-4d99-895a-85d6b7de4ac7'}"
```

Requires the station simulation, the `movement_controller` (active), the `cameras` node
and the `station_workplace` node (or a publisher of its `workplace` topic and the TF
`carrier` -> `estimated_product_origin`). The real pylon camera driver has no `get_image`
action, so the localizer currently works with the simulated camera only.
