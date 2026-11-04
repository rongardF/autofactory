# product_carrier

ROS 2 lifecycle nodes that control product carrying hardware (conveyor line, trolley lifter, …). Every carrier node offers the same interface so carriers are interchangeable: a product is *injected* into the robot workplace slot, the robot works on it, and the product is *ejected* again.

Currently implemented: **`conveyor`** (simulated backend; hardware backend is a stub).

## Node `conveyor`

```bash
ros2 run product_carrier conveyor
# or
ros2 launch product_carrier conveyor.launch.py
```

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `simulated` | bool | `true` | Use the simulated (Gazebo) backend instead of real hardware |

### Interface

| Name | Kind | Type | Description |
|---|---|---|---|
| `~/inject_product` | service | `std_srvs/srv/Trigger` | Move a new product into the workplace slot |
| `~/eject_product` | service | `std_srvs/srv/Trigger` | Move the product out of the workplace slot |
| `~/product_in_place` | topic | `std_msgs/msg/Bool` | `true` while a product is in the workplace. Reliable, transient local, depth 1 (latched). Published as `false` on activation, then only on change |

### Lifecycle

The node never transitions by itself; drive it externally:

```bash
ros2 lifecycle set /conveyor configure   # reads parameters, selects backend
ros2 lifecycle set /conveyor activate    # initialises backend, creates services/topic
ros2 lifecycle set /conveyor deactivate  # removes services/topic, de-initialises backend
ros2 lifecycle set /conveyor cleanup     # drops configuration and backend
ros2 lifecycle set /conveyor shutdown
```

A failed activation rolls back everything it created and leaves the node Inactive. An error during a transition releases all resources and returns the node to Unconfigured.

### Simulation

- `inject_product` / `eject_product` are no-ops returning `success: true`; products are moved manually in Gazebo.
- Presence is detected by a single-ray `gpu_lidar` (0.2 m range) mounted across the carrier path at the workplace slot. Any in-range return means a product is in place.
- The node subscribes to `presence_sensor_scan` (`sensor_msgs/msg/LaserScan`); the launch file remaps it to the bridged Gazebo topic.

Caveats:
- The lidar sees visual geometry: place it so the ray does not hit the conveyor itself.
- Anything crossing the beam (e.g. the robot arm) is reported as a product in place.
- The Gazebo world must load the Sensors system.

### Hardware

Not implemented yet: activation fails with `Conveyor hardware mode is not implemented`.

## Launch arguments (`conveyor.launch.py`)

| Argument | Default | Description |
|---|---|---|
| `simulated` | `true` | Simulated or real hardware; also sets `use_sim_time` |
| `node_name` | `conveyor` | Node name |
| `tf_prefix` | `''` | Must match the URDF macro `tf_prefix`; selects the Gazebo topics to bridge |

With `simulated:=true` a `ros_gz_bridge` is started for:

| Gazebo topic | ROS topic | Type |
|---|---|---|
| `/<tf_prefix>conveyor/presence_sensor` | same | `sensor_msgs/msg/LaserScan` |
| `/<tf_prefix>conveyor/joint_state` | `/joint_states` | `sensor_msgs/msg/JointState` |

## URDF macro (`urdf/conveyor.urdf.xacro`)

```xml
<xacro:include filename="$(find product_carrier)/urdf/conveyor.urdf.xacro"/>

<xacro:conveyor
  parent="station"
  simulated="true"
  mount_origin_xyz="1.2 -0.475 0"
  mount_origin_rpy="0 0 0"
  conveyor_length="0.6"
/>
```

| Argument | Default | Description |
|---|---|---|
| `parent` | required | Link the conveyor is mounted on (prefixed with `tf_prefix`) |
| `simulated` | `true` | Add Gazebo elements and make the carrier joint prismatic |
| `mount_origin_xyz` | `0 0 0` | Position of `conveyor_origin` relative to `parent` |
| `mount_origin_rpy` | `0 0 0` | Orientation of `conveyor_origin` relative to `parent` |
| `conveyor_length` | `0.6` | Carrier travel length [m] |
| `update_rate_hz` | `10.0` | Presence lidar update rate [Hz] |
| `tf_prefix` | `''` | Prefix for all links, joints and Gazebo topics |

Links: `conveyor_origin` (meshes, inertial), `carrier_base`, `carrier` (attach objects here).

Joints: `<parent>_conveyor_joint` (fixed), `origin_base_joint` (fixed), `base_carrier_joint`:
- `simulated=true`: prismatic along X of `carrier_base`, range `[-conveyor_length, 0]`, starts at 0 and moves in -X, passive (no controller). Its state is published by the Gazebo JointStatePublisher system and bridged to `/joint_states`.
- `simulated=false`: fixed at `-conveyor_length`.

## SRDF macro (`srdf/conveyor.srdf.xacro`)

Invoke from the assembled SRDF to disable collisions between `conveyor_origin` and its parent:

```xml
<xacro:include filename="$(find product_carrier)/srdf/conveyor.srdf.xacro"/>
<xacro:conveyor_srdf parent="station"/>
```
