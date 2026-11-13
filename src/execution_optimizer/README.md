# execution_optimizer

ROS 2 (Jazzy) package whose nodes decide **in which groups the patterns of an operation
should be executed**, so that all patterns are executed efficiently while images that satisfy
fixed quality criteria can be captured around every group.

Intended consumer workflow (outside this package):

1. Group the operation's patterns with the optimizer.
2. For each group:
   1. Move the camera to the group's image capture pose(s) and capture image(s).
   2. Execute all patterns of the group.
   3. Move the camera to the same pose(s) and capture image(s) again.

There is one node per operation type, because each type has its own criteria. All nodes expose
the same interface shape with operation specific request messages. Currently implemented:
`dispensing_optimizer`.

## Architecture

```
execution_optimizer/
├── msg/PatternGroup.msg
├── srv/DispensingOptimizePatterns.srv
└── execution_optimizer/
    ├── dispensing_optimizer.py            # concrete node + main()
    ├── interface/optimizer_base.py        # abstract lifecycle node (shared behaviour)
    ├── model/                             # frozen Pydantic DTOs
    ├── enumerator/                        # enums
    ├── service/
    │   ├── camera_info_provider.py        # GetImage client, init / deinit
    │   └── dispensing_grouping_service.py # grouping heuristic (pure Python, no ROS)
    └── util/
        ├── geometry.py                    # line / plane fit, minimum-area rectangle
        └── camera_pose.py                 # camera orientation, LP feasibility, centring
```

A new operation node subclasses `OptimizerBase` and provides the configuration DTO, the grouping
service, the service interface and the pattern-to-points conversion.

## Interface

`~/optimize_patterns` (`execution_optimizer/srv/DispensingOptimizePatterns`) — exists only while the
node is **active**.

Request: `repositories/OperationRepositoryDispensingPattern[] patterns`.

Response:

| Field | Meaning |
|---|---|
| `groups` | `execution_optimizer/PatternGroup[]` in creation order; empty on failure. |
| `optimization_time_s` | Wall time of the optimization, set in both cases. |
| `success` | `false` on any failure; no exception crosses the service boundary. |
| `error_message` | Empty on success; otherwise the cause, naming pattern ids where applicable. |

`PatternGroup`: `pattern_ids` (UUID4 strings, request order) and `image_capture_poses`
(`geometry_msgs/PoseStamped[]`, `header.frame_id = product_frame_id`, always exactly one pose for
dispensing). Every request pattern is in exactly one group.

Client used internally: the `GetImage` action of the `cameras` package at
`<camera_node_name>/get_image` (resolved in the node's namespace).

## Parameters

| Name | Type | Default | Meaning |
|---|---|---|---|
| `product_frame_id` | string | `product_origin` | Frame of all pattern poses and of the output poses. |
| `feature_resolution_range` | double[2] | `[3.33, 4.16]` | Inclusive `[low, high]` px/mm, `0 < low <= high`. The default matches 400–500 mm for the simulated camera (fx ≈ 1663 px); a real camera must override it. |
| `max_angle_deviation` | double | `20.0` | Degrees, `0 <= v < 90`. Maximum angle between any ROI normal of a group and the group's mean normal. |
| `operational_area` | int[] | `[]` | Empty (whole image) or inclusive `[x_min, y_min, x_max, y_max]` pixels, top-left origin. Checked against the image size on activation. |
| `camera_node_name` | string | `basler_camera` | Camera node; action name `<camera_node_name>/get_image`. |
| `planarity_tolerance_mm` | double | `0.5` | Maximum distance of pattern points from the fitted plane / line. |
| `camera_info_timeout_s` | double | `5.0` | Timeout for the camera action server and the `GetImage` result on activation. |

Parameters are read and validated in `on_configure`; invalid values fail the transition.

## Behaviour

**Pattern points.** The start position, every movement target position and every circular
movement point. All poses must be in `product_frame_id` (no TF lookups); arcs are not sampled.

**Region of interest (ROI).** The minimum-area rectangle of the points in the pattern's plane. The
plane normal comes from a least-squares fit and points up (positive product Z). Points further than
`planarity_tolerance_mm` from the plane fail the request. Collinear or single-point patterns have a
zero-area ROI and the product +Z as normal; a horizontal normal fails the request.

**Camera model.** Pinhole without distortion; only `fx, fy, cx, cy, width, height` of the camera's
`CameraInfo` are used. Poses are poses of the camera TCP (projection centre). The optical axis is
anti-parallel to the group's mean ROI normal and the roll is fixed so that image-right follows the
product X axis. The TCP frame follows the Gazebo camera convention (+X optical axis, +Y image-left,
+Z image-up).

**Group feasibility.** A set of patterns can share one image if every ROI normal is within
`max_angle_deviation` of the mean normal and a camera position exists for which every ROI corner has
`fx/Z` and `fy/Z` (`Z` = depth along the optical axis) inside `feature_resolution_range` and projects
into the operational area. For a fixed orientation these are linear constraints on the camera
position and checking the ROI corners is sufficient; a linear program (HiGHS) maximises the margin to
all constraints, and the pose is the analytic centre of the feasible region.

**Grouping.** Minimising the number of groups is NP-hard, so a deterministic heuristic is used:
first-fit over several fixed pattern orderings (by ROI area and by sweeps along the product axes),
each followed by a pass that tries to dissolve the smallest groups into the others. The result with
the fewest groups wins. Identical requests give identical results. Each final group is re-checked
independently against all criteria before it is returned.

**Failures** (`success=false`, `groups` empty): empty request, ids that are not UUID4 or not unique,
poses in another frame, non-planar patterns, horizontal normals, and patterns that cannot be imaged
with the configured criteria on their own (the ids are listed).

## Lifecycle

| Transition | Actions |
|---|---|
| configure | Read parameters into a frozen config DTO; create (but do not initialize) the camera-info provider and the grouping service. |
| activate | Fetch the camera intrinsics with the `GetImage` action (async + callbacks, bounded by `camera_info_timeout_s`), validate `operational_area` against the image size, create `~/optimize_patterns`. Any failure tears everything down and the node stays inactive. |
| deactivate | Destroy the service and the action client; drop the cached intrinsics. |
| cleanup | Delete the internal services and the config. |
| shutdown | Releases everything that is still held, from any state. |

The node must be spun by a `MultiThreadedExecutor` (done in `main`): the camera action callbacks run
while the activation transition waits for them.

## Build and run

```bash
colcon build --symlink-install --packages-select execution_optimizer
source install/setup.bash
ros2 launch execution_optimizer dispensing_optimizer.launch.py
ros2 lifecycle set /dispensing_optimizer configure
ros2 lifecycle set /dispensing_optimizer activate
```

The launch file defaults `camera_node_name` to `cameras/basler_camera` (the simulated camera's
namespace) and `camera_info_timeout_s` to `10.0`. Python dependencies: `numpy`, `scipy`, `pydantic` v2.

## Out of scope

Robot reachability, collision checks, motion planning, camera motion, image capture for processing,
lens-distortion modelling, ordering / travel-time optimisation.
