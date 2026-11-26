# world_manager

Owns all manipulation of the Gazebo world and the MoveIt 2 planning scene. Other packages are clients and never talk to
Gazebo or to `/apply_planning_scene` directly.

| Node | Status | Runs when |
|---|---|---|
| `gazebo_world_manager` (lifecycle) | implemented | `simulated:=true` only |
| `moveit2_world_manager` (lifecycle) | implemented | always |

Both nodes are started unconfigured by [world_manager.launch.py](launch/world_manager.launch.py); the MoveIt node
starts in both real and simulated modes. An external lifecycle orchestrator must configure and activate them before
consumers activate. The launch file does not initiate lifecycle transitions.

The managers' public services and the Gazebo `~/models` topic exist only while the corresponding node is active.
Deactivation removes them; reactivation creates them again. Lifecycle control and ROS node-management interfaces
remain available to the orchestrator.

## gazebo_world_manager

Talks to Gazebo through the gz-transport Python bindings (no `ros_gz_bridge`), only through the `/blocking` service
variants. Services (under `/gazebo_world_manager/`): `spawn_models`, `delete_models`, `set_model_pose`,
`set_model_attachment`, `list_models`. Every response carries `success` and `error_message`.

gz service requests run in a helper process (`python -m world_manager.services.gz_request_process`, started on
the first request and inheriting the `GZ_*` environment). The gz-transport Python binding keeps the GIL during a
blocking request, while subscription callbacks need it on gz-transport's single receive thread. A joint state event
that arrives during a request (for example the carrier's `attached` event while the product is being created)
would otherwise hold back the reply until the request times out.

State topic `~/models` (`world_manager/ModelList`, reliable + transient-local) is republished only when the list changes.

### Deleting models

`delete_models` takes model ids or an owner. A model that another model still holds through a non-detached
DetachableJoint cannot be deleted alone: the call is rejected before anything is deleted. The exception is a holder
spawned with `delete_with_child=true` (`ModelSpec`): it is added to the delete set, whatever its owner (transitively),
detached and removed before the model it holds. The dispensed material of the endtools is spawned this way (each blob
holds the previous one, the first holds the product), so deleting a product also removes the material on it. If a
cascaded holder can neither be detached nor removed, the model it holds
is kept and the call fails. Cascaded models do not use up `operation_timeout_sec`.

Parameters: `world_name` (required), `world_frame`, `service_timeout_sec`, `tf_timeout_sec`, `confirm_timeout_sec`,
`republish_period_sec`, `operation_timeout_sec`, `rollback_timeout_sec` (see [config](config/world_manager.yaml)).

### World preconditions

The world SDF must load `gz::sim::systems::UserCommands` (create/remove/set_pose) and
`gz::sim::systems::SceneBroadcaster` (`scene/info`). Activation fails with a clear error if `scene/info` does not answer.

### Assumptions and limits

- Gazebo and the manager share one container (`GZ_IP=127.0.0.1`, no `GZ_PARTITION`). Running Gazebo elsewhere
  requires matching both variables on `gazebo_world_manager`.
- The registry is in memory. After a manager restart, models spawned before the restart are unmanaged and are never
  touched or adopted; reset or reload the Gazebo world together with the restart.
- The initial `attached` state of a DetachableJoint is assumed (reported with `state_confirmed=false`) when its
  one-time event is missed. A successful spawn is not proof of physical attachment.
- A joint is `controllable` only if its resolved attach/detach/state topics are unique among all joints;
  `set_model_attachment` fails on other joints.

## moveit2_world_manager

Owns all planning-scene collision-object and Allowed Collision Matrix (ACM) changes. It exposes
`apply_world_changes`, `attach_object` and `list_objects` under `/moveit2_world_manager/`. Every response carries
`success` and `error_message`.

`apply_world_changes` makes one planning-scene diff for additions/replacements, removals and ACM changes. Meshes are
loaded before reading the scene, so an invalid mesh makes no changes. Existing world and attached copies are removed
when an id is replaced. ACM rows and columns for removed/replaced object ids are dropped before requested pairs are
applied. `attach_object` moves an existing object's geometry between the world and a robot link and requires the pose
frame to be empty or match that destination.

Parameters: `get_scene_service`, `apply_scene_service`, `service_timeout_sec`, `planning_frame`. The manager serializes
its own scene updates, including ACM read-modify-write; other writers to `/apply_planning_scene` (such as RViz or
scripts) are not serialized and can still race. The manager should be the only ACM writer.

## Client library

`world_manager.client.GazeboWorldClient` and `MoveIt2WorldClient` provide blocking, typed wrappers for consumer nodes.
Construct them with the consumer node and a callback group that permits concurrent service responses. The Gazebo
client is created only when the consumer's `simulated` parameter is true; it has no no-op mode. Client calls use
`call_async` with bounded service availability and response waits. A timeout or cancellation abandons only the local
wait; the manager may still finish the request, so read the manager's list service if the outcome is uncertain.
Client timeouts should exceed the relevant manager's operation timeout.

`world_manager.utils.expand_xacro(path, mappings)` is a small helper for consumers to expand xacro before sending SDF
strings to `GazeboWorldClient.spawn`.

## Tests

```bash
python -m pytest src/world_manager/tests/unit -v
python -m pytest src/world_manager/tests/integration -v   # marker `gazebo`; needs `gz`, skipped otherwise
```

Tests are never installed. `tests/integration/assets/` holds the spike world, a two-joint tool, a one-joint carrier and
the spike scripts.
