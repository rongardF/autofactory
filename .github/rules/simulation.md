# Simulation Conventions — RViz + Gazebo

## Target Simulation Stack

The full workstation solution must be simulatable with all hardware components:

| Hardware | Simulation Method |
|----------|------------------|
| UR10e robot arm | `ur_robot_driver` fake hardware interface |

**Visualization:** RViz — robot state, MoveIt2 planning scene, camera feeds, topic monitors  
**Physics:** Gazebo **Harmonic** (the Gazebo version paired with ROS 2 Jazzy on Ubuntu 24.04; Ignition Fortress is for ROS 2 Humble — do not confuse them)

---

## use_sim_time

All nodes **must** respect the ROS 2 `use_sim_time` parameter:

```python
from rcl_interfaces.msg import ParameterDescriptor

class MyNode(LifecycleNode):
    def __init__(self, node_name: str = 'my_node') -> None:
        super().__init__(node_name)
        # use_sim_time is set by the launch file — declare it here for documentation
        self.declare_parameter('use_sim_time', False, ParameterDescriptor(description='Use ROS 2 simulation time instead of wall clock'))
        # rclpy.clock automatically uses sim time when use_sim_time=True is set
        # via the launch file — no manual handling needed in most nodes
```

In launch files, always pass `use_sim_time` as a launch argument and forward to all nodes:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='movement_controller',
            executable='ur_movement_controller',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # ... all other nodes also receive use_sim_time
    ])
```

**Rules:**
- `use_sim_time` is set at the launch layer — never hardcoded in node source
- Never use `time.time()` or Python's `datetime.now()` in ROS 2 nodes; use `self.get_clock().now()`
- `rclpy.clock.Clock` with `ClockType.ROS_TIME` automatically handles sim vs wall time when `use_sim_time` is set

---

## Simulation vs Real Hardware — Launch Layer Branching

Node source code **must not** branch on simulation vs real hardware. All branching happens in launch files:

```python
# WRONG — node code branching on sim
class DispenserNode(LifecycleNode):
    def dispense(self, volume: float) -> None:
        if self.get_parameter('use_sim').value:
            self.get_logger().info(f'[SIM] Dispensing {volume}ml')
        else:
            self._hardware.actuate(volume)  # real hardware

# CORRECT — node is hardware-agnostic; launch file selects the real or fake implementation
class DispenserNode(LifecycleNode):
    def dispense(self, volume: float) -> None:
        # calls the hardware interface — either real or fake, set by launch
        self._hardware_interface.actuate(volume)
```

The launch file selects which `hardware_interface` plugin is loaded:

```python
# sim.launch.py
Node(package='tools', executable='dispenser_node',
     parameters=[{'hardware_plugin': 'tools_fake/FakeDispenserHardware'}])

# real.launch.py
Node(package='tools', executable='dispenser_node',
     parameters=[{'hardware_plugin': 'tools_hw/DispenserHardware'}])
```

---

## UR Robot Driver — Fake Hardware

For simulation, use `ur_robot_driver`'s built-in fake hardware interface. Consult the official docs for the correct launch argument:
https://docs.universal-robots.com/Universal_Robots_ROS2_Driver/

```python
# In simulation launch files, pass the fake hardware flag
# (exact argument name — check docs, do not guess)
# Example pattern — verify parameter name from official docs:
DeclareLaunchArgument('use_fake_hardware', default_value='true'),
```

The fake hardware interface publishes joint states and accepts trajectory goals without a real robot connected. MoveIt2 planning and execution work identically against both fake and real hardware.

---

## Gazebo Integration

- Use `ros_gz_bridge` (ROS 2 ↔ Gazebo bridge) for sensor topics (camera images, IMU, contact sensors)
- All Gazebo world models (.sdf) live in `simulation/worlds/`
- Robot URDF/xacro files used for MoveIt2 are also used to generate the Gazebo model — single source of truth
- It must be possible to add/remove models programmatically

### Bridge configuration example

```yaml
# ros_gz_bridge parameter file
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/world/workstation/model/camera/link/camera_link/sensor/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
```

---

## Simulation Checklist (per phase)

When implementing any phase with hardware-commanding code:

- [ ] Node accepts `use_sim_time` parameter (declared, not assumed)
- [ ] Node does not branch on `use_sim` — uses hardware interface plugin instead
- [ ] Hardware interface has a fake/stub implementation loadable by the simulation launch
- [ ] All time references use `self.get_clock().now()`, not `time.time()`
- [ ] Node is included in `sim_basic.launch.py` (or planned for the correct sim launch target)

---

## Troubleshooting

### Gazebo `gazebo-4` segfault when loading a custom COLLADA (`.dae`) mesh

**Symptom**

Launching Gazebo (e.g. `ros2 launch ur_gazebo_simulation ur_sim_control.launch.py ...`)
crashes with a line like:

```
[gazebo-4] Segmentation fault (Address not mapped to object [0x........])
```

The crash is often **intermittent** and only happens when the GUI/rendering is active
(`gazebo_gui:=true`). Commenting out the `<visual>` block that references the custom
mesh makes it "go away" — which is misleading. The fault address sometimes looks like
ASCII text (e.g. `0x36323732` = "2762"), a hint that raw data is being read as a pointer.

**Root cause**

`gz-common`'s COLLADA loader (`ColladaLoader::LoadPolylist`) **null-derefs when a
`<polylist>` has its `VERTEX` and `NORMAL` inputs sharing the same `offset="0"`.**
Meshes exported by browser/JS tools (THREE.js `GLTFExporter` → Assimp) commonly emit
this shared-offset form. The stock UR meshes work because they give `NORMAL` its own
`offset="1"`.

Only the **render** path hits this — the physics server (`gz sim -s -r`) and collision
`.stl` load fine, which is why it presents as GUI-only and intermittent.

**How to diagnose (fast)**

1. Reproduce in isolation with a real display, watching for the crash within ~12 s:
   ```bash
   export DISPLAY=:1   # or your virtual/X display
   cat > /tmp/t.sdf <<'EOF'
   <?xml version="1.0"?>
   <sdf version="1.9"><world name="t"><model name="m"><static>true</static>
   <link name="l"><visual name="v"><geometry><mesh>
   <uri>file:///ABS/PATH/to/your_mesh.dae</uri>
   </mesh></geometry></visual></link></model></world></sdf>
   EOF
   gz sim -r -v 2 /tmp/t.sdf         # crashes ~12s in if the mesh is bad
   ```
   A backtrace (install `gdb`; Gazebo auto-prints a stack on crash) will show:
   `ColladaLoader::LoadPolylist → LoadGeometry → LoadNode → MeshManager::Load → loadMesh`.

2. Confirm the mesh structure is the shared-offset form:
   ```bash
   grep -E '<input .*semantic="(VERTEX|NORMAL)"' your_mesh.dae
   # BAD  -> both show offset="0"
   # GOOD -> NORMAL shows offset="1"
   ```

**Fix**

Rewrite the `<polylist>` so `NORMAL` has its own offset and the index array (`<p>`)
carries a per-vertex normal index. When normals align 1:1 with positions (the common
case for these exports), the normal index equals the vertex index, so each index `i`
in `<p>` becomes the pair `i i`:

```python
# 1) NORMAL input: offset="0" -> offset="1"
# 2) <p> stride 1 -> 2:  "a b c ..."  ->  "a a b b c c ..."
```

Then verify `sum(vcount) * stride == len(p)` and `max(index) < accessor_count`, and
re-run the isolation repro above (should survive several launches with no crash).
Apply the corrected mesh to **both** `src/` and the built `install/` copy (or rebuild
with `colcon build --symlink-install`).

> Prefer fixing the mesh over exporting to OBJ, so the DAE keeps its `up_axis` and the
> URDF `<visual>` `origin`/`rpy` stays valid.
