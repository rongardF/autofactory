# Building

Navigate into devcontainer workspace root and run:

bash
```
colcon build
```

# Launching

Run following command to launch the station launch system in simulation with Gazebo GUI:

bash
```
ros2 launch station station.launch.py simulated:=true model:=ur10e
```

Run following command to launch the station launch system with real hardware (no Gazebo GUI):

bash
```
ros2 launch station station.launch.py simulated:=false ip_address:=<robot-ip-address> model:=ur10
```

# View camera image

To view camera image (real or simulated) run:

bash
```
ros2 run rqt_image_view rqt_image_view /cameras/basler_camera/image_raw
```

# Laser cross sensor triggers

To read the laser cross sensor outputs (real or simulated):

bash
```
ros2 topic echo /laser_sensors/captron_orl2/x_axis_triggered
ros2 topic echo /laser_sensors/captron_orl2/y_axis_triggered
```

# Automated tool changer

There is an automated tool changer implemented to switch between endtools. Below is quick start and demo.

Enable the nodes (will also spwn the endtools):

bash
```
ros2 lifecycle set /movement_controller configure
ros2 lifecycle set /movement_controller activate
ros2 lifecycle set /tools_manager configure
ros2 lifecycle set /tools_manager activate
```

To mount the tool on SLOT 1 (tool serial number 'abc123'):

bash
```
ros2 action send_goal /tools_manager/mount_tool tools_manager/action/MountTool "{tool_sn: 'abc123'}"
```

To un-mount the tool to SLOT 1 (tool serial number 'abc123'):

bash
```
ros2 action send_goal /tools_manager/unmount_tool tools_manager/action/UnmountTool "{tool_sn: 'abc123'}"
```

# Example scripts


Following commands must be executed to activate the '/movement_controller' lifecycle node before any movment can be performed:

bash
```
ros2 lifecycle set /movement_controller configure
ros2 lifecycle set /movement_controller activate
```

There are example Python scripts located in `/examples` directory to illustrate how to perform various type of movments. To run them execute in terminal in devcontainer root folder:

bash
```
python3 examples/movement_to_laser_cross_sensor.py
```

# ROS2 packages

## Gazebo models

Each ROS 2 package **owns its Gazebo models in its own source tree** and
installs them with a normal `install(DIRECTORY ...)` rule. A shared CMake macro,
`gz_register_models()`, then registers that already-installed directory onto
`GZ_SIM_RESOURCE_PATH` so Gazebo can resolve `model://...` URIs. Models are
**never copied to a central location** and **never installed twice** — the macro
only wires up a runtime environment hook, it does not install anything itself.

### The shared macro

The macro lives at `docker/cmake/gz_register_models.cmake` and is **baked into
the container image** (not vendored into each package). See `docker/Dockerfile`:

```dockerfile
COPY docker/cmake/gz_register_models.cmake /usr/local/share/cmake/gz/
ENV GZ_CMAKE_MODULE_DIR=/usr/local/share/cmake/gz
```

`gz_register_models(<relative-share-path>)` generates a bash environment hook
(`gz_resource_path.sh`) and registers it via `ament_environment_hooks()`. At
runtime, sourcing the workspace overlay prepends
`$AMENT_CURRENT_PREFIX/share/<PROJECT_NAME>/<relative-share-path>` to
`GZ_SIM_RESOURCE_PATH`. Because it uses `$AMENT_CURRENT_PREFIX`, the path stays
relocatable (works from a dev `install/` prefix or a stripped production
prefix). Duplicate entries are harmless to Gazebo.

### Using it in a package

Two steps in the package's `CMakeLists.txt` — install the directory the normal
way, then register it (no second install):

```cmake
# 1. Install model assets the normal way
install(DIRECTORY config launch urdf srdf DESTINATION share/${PROJECT_NAME})

# 2. Register the model dir on GZ_SIM_RESOURCE_PATH
list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
include(gz_register_models)
gz_register_models(config/model)   # dir that directly contains the model:// folders
```

The argument is the path (relative to `share/<PROJECT_NAME>/`) to the directory
that **directly contains** the `model://` folders. Current usage:

| Package | Call | Model directory contains |
|---------|------|--------------------------|
| `cameras` | `gz_register_models(config/model)` | `basler/` |
| `station` | `gz_register_models(config/model)` | `station/`, `textured_ground/`, ... |
| `laser_sensors` | `gz_register_models(model)` | laser sensor model(s) |

### Runtime flow

1. `colcon build --symlink-install` — `install()` places models under
   `install/<pkg>/share/<pkg>/...`; the macro drops the env hook into the
   package's install prefix.
2. `source install/setup.bash` — each package's hook prepends its model dir to
   `GZ_SIM_RESOURCE_PATH`.
3. Gazebo resolves `model://basler`, `model://station`, etc. across all
   packages.