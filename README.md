# Autofactory

## Initialize the development environment

### Prerequisites

Install these on the host:

- Docker with Linux-container support and a local Docker daemon.
- VS Code with the Dev Containers extension.
- Python 3, available as `python3` on PATH.

On **Windows**, open the repository from WSL2 using VS Code's WSL extension.
Enable Docker Desktop integration for that WSL distribution. WSLg provides
native application windows when available.

On **Linux**, install `xauth` if your native display requires authentication.
For NVIDIA acceleration, the host needs a working driver and NVIDIA Container
Toolkit. CPU rendering is available without GPU setup.

On **macOS**, use the browser desktop. Apple Silicon uses `linux/amd64`
emulation for the existing x86-only Pylon SDK installer, so builds and
simulation will be slower.

### Open the container

1. Start Docker and open the repository in VS Code.
2. Run **Dev Containers: Reopen in Container** from the Command Palette.
3. Wait for the image build to install dependencies and the Pylon SDK.
4. Open a VS Code terminal inside the container and build the workspace below.

After changing the Dockerfile or Dev Container configuration, use
**Dev Containers: Rebuild Container**.

Terminals and builds run as `ros`, with `sudo` available for system packages.
Display and GPU selection are automatic; see [Displays and rendering](#displays-and-rendering)
for platform behavior and overrides.

## Build the workspace

Run inside the container:

```bash
cd /workspaces/autofactory
colcon build
```

After building, load ROS 2 and the workspace in each terminal before launching
nodes or running `ros2` commands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

The first command loads ROS 2 Jazzy; the second makes workspace packages,
including `station`, available.

## Launch the station

### Simulation with Gazebo and RViz

```bash
ros2 launch station station.launch.py simulated:=true gazebo_gui:=true rviz:=true model:=ur10e
```

The windows appear on the native display or browser desktop selected at startup.
For the browser desktop, open
<http://localhost:6080/vnc.html?autoconnect=1&resize=scale>.

### Simulation without the Gazebo GUI

```bash
ros2 launch station station.launch.py simulated:=true gazebo_gui:=false model:=ur10e
```

### Real hardware

Replace `<robot-ip-address>` with the robot's address:

```bash
ros2 launch station station.launch.py simulated:=false ip_address:=<robot-ip-address> model:=ur10
```

## Work with cameras and sensors

### View a camera image

For a real or simulated camera:

```bash
ros2 run rqt_image_view rqt_image_view /cameras/basler_camera/image_raw
```

### Read laser cross sensor triggers

For real or simulated sensors:

```bash
ros2 topic echo /laser_sensors/captron_orl2/x_axis_triggered
ros2 topic echo /laser_sensors/captron_orl2/y_axis_triggered
```

## Run movement examples

Activate the `/movement_controller` lifecycle node before performing movement:

```bash
ros2 lifecycle set /movement_controller configure
ros2 lifecycle set /movement_controller activate
```

The `examples/` directory contains Python movement examples. Run them from the
workspace root inside the container:

```bash
python3 examples/movement_to_laser_cross_sensor.py
```

## Displays and rendering

### Automatic display selection

The Dev Container selects a display before starting:

| Host | Default display | Rendering |
|------|-----------------|-----------|
| Linux with an X11/XWayland display | Native windows | Hardware OpenGL when available, otherwise Mesa CPU |
| Windows / WSL2 with WSLg | Native Windows windows | WSLg / Mesa D3D12 when available, otherwise CPU |
| macOS, or a host without a usable native display | Browser desktop | Mesa CPU |

For the browser desktop, open
<http://localhost:6080/vnc.html?autoconnect=1&resize=scale>.
Launch Gazebo and RViz from the VS Code container terminal using the usual ROS
launch commands; their windows appear on the selected desktop. The browser
contains a window manager, so both applications can be moved and resized.
The browser endpoint is published only on localhost; do not expose or publicly
forward port 6080, since this local development desktop has no password.

The browser path deliberately uses CPU rendering; passing an NVIDIA GPU into an
Xvfb session alone does not provide GPU OpenGL. macOS Docker containers cannot
use the Mac GPU through this setup. GPU acceleration is supported through native
Linux/WSLg displays. Ogre2 remains enabled in both modes; software rendering may
be substantially slower for simulated cameras and laser sensors.

### Override display or rendering selection

To override detection, set these variables in the environment of the process
that opens VS Code, then **Rebuild Container**:

```bash
# Browser desktop on any platform
AUTOFACTORY_DISPLAY=browser code .

# Native display with CPU rendering (Linux / WSLg)
AUTOFACTORY_DISPLAY=native AUTOFACTORY_GPU=cpu code .
```

`AUTOFACTORY_DISPLAY` accepts `auto` (default), `native`, or `browser`;
`AUTOFACTORY_GPU` accepts `auto` (default) or `cpu`. Fully quit existing VS Code
processes first if they were started with a different environment. These are
host variables, not entries in the repository's `.env` file.

### Troubleshooting

Inspect the active display and renderer:

```bash
echo "$DISPLAY"
glxinfo -B
cat /run/autofactory/rendering.sh
```

Desktop logs are in `/tmp/autofactory-desktop/`.
`/run/autofactory/rendering.sh` is generated at startup and loaded by terminals
so applications use the selected display and rendering settings.

If earlier root sessions
created build artifacts you cannot overwrite, repair only those directories
from a container terminal (avoid recursively changing the whole repository):

```bash
sudo chown -R "$(id -u):$(id -g)" build install log
```

## Dev Container implementation

### User and permissions

The Dev Container uses the `ros` account for VS Code, terminals, and workspace
builds. On Linux/WSL, `updateRemoteUserUID` matches the host UID/GID so new files
retain normal host access. The display entrypoint remains root to initialize
services and device groups; rendering settings and X11 authentication are made
available to `ros`. System dependencies are installed during the image build.
Passwordless `sudo` remains available for development. Users can add personal
shell settings to their `.bashrc`.

The container entrypoint and Bash shells source
`/usr/local/share/autofactory/setup.bash` (from `docker/setup.bash`) to load
Python, ROS, the workspace overlay, and generated rendering settings. The image
configures interactive and login shell hooks; the Dev Container uses `BASH_ENV`
for noninteractive Bash commands. No post-create script is needed.

### Codex and robotics extensions

Both Dev Container configurations install Codex (`openai.chatgpt`) and the
Robotics Developer Environment pack (`Ranch-Hand-Robotics.rde-pack`), replacing
the deprecated Microsoft ROS extension.

Your host user's `~/.codex` is mounted at `/home/ros/.codex` and used as
`CODEX_HOME`. Codex chats, settings, and file-based login survive container
rebuilds and remain personal to your host account, outside this repository.
On Windows, reopen from WSL so this uses your WSL home directory. The host
directory is created automatically if missing. This also shares Codex state
with Codex running on that host account.

After **Dev Containers: Rebuild Container**, open Codex in the VS Code sidebar
and sign in with your ChatGPT account. For persistent login in a container,
set the following top-level option in your personal `~/.codex/config.toml`
(create the file if needed), then sign in:

```toml
cli_auth_credentials_store = "file"
```

Host keyring credentials are not available through the directory mount.
Treat `~/.codex/auth.json` as a password; never commit or share it.
Existing chats stored only in an old container must be copied to the host's
`~/.codex` before removing that container; this setup does not migrate them.

### Native display access

Linux native mode forwards `/tmp/.X11-unix` and copies the current display's X11
cookie using host `xauth` when available. Install `xauth` on Linux if the display
requires authentication. No global `xhost +` permissions are needed. If native
X11 authentication or OpenGL fails, startup falls back to the browser desktop.
WSLg supplies the X11 socket and, when available, `/dev/dxg` and WSL driver
libraries. On Linux, NVIDIA requires a working host driver and NVIDIA Container
Toolkit; Intel/AMD use `/dev/dri`. A graphics probe inside the container checks
actual OpenGL access, rather than assuming GPU visibility implies rendering.

### Generated configuration and networking

Host detection generates ignored files under `.devcontainer/.generated/`,
including an X11 cookie when required. Never commit this directory. This setup supports a local Docker daemon, uses container networking for
ROS nodes. The root `docker-compose.yml` is reserved for production.
ROS discovery across the host or other machines requires separate DDS/network
configuration.

### Container build cache

The Dockerfile uses named stages, each inheriting the previous stage:

- `base`: pinned ROS image, system packages, and display tools.
- `ros-dependencies`: workspace ROS dependencies and RViz.
- `camera-sdk`: Pylon and Blaze SDKs.
- `python-dependencies`: Python virtual environment and pinned packages.
- `development`: graphics configuration, the `ros` account, and entrypoints.

Both Compose configurations build the `development` target. To build an earlier
stage independently, use, for example:

```bash
docker build --platform linux/amd64 -f docker/Dockerfile --target ros-dependencies -t autofactory:ros-dependencies .
```

Dependencies remain in separate cached layers. Rebuilding after a shell or
display script edit reuses those dependency layers. Shell hooks are configured
during the image build; there is no post-create installation step.

The base image is pinned by digest, direct apt dependencies use explicit
versions in `docker/Dockerfile`, Python requirements include transitive version
pins, and SDK archives are checked against fixed SHA-256 hashes. BuildKit caches
apt, pip, and SDK downloads when an installation step needs to run again.

To update dependencies, edit the relevant Dockerfile versions or requirements
and rebuild. When adding a dependency to `package.xml`, also add its apt package
and version to the ROS dependency layer. There is no generated lock workflow.
Apt's transitive dependencies still come from the configured repositories, so
this provides controlled version pins, not byte-for-byte reproducible images.
Unavailable pinned versions fail the build rather than selecting a newer direct
dependency.

The workspace remains mounted from your local checkout, including `build/`,
`install/`, and `log/`. A container rebuild preserves those directories. Run
`colcon build` after source changes; the Docker build does not compile the
workspace. Compiler or ABI changes can still require a clean workspace build.

## Production Compose

The root `docker-compose.yml` runs only the production workstation. It has no
development service, source mounts, or display/GPU setup. Development uses
`.devcontainer/compose.yaml`.

Provide a prebuilt `workstation:prod` image containing the compiled workspace,
the `ros` account (UID/GID 1000), the ROS entrypoint, and a production launch
command. The current Dockerfile builds the development environment; tagging it
as `workstation:prod` does not create a deployable application image.

Once that image is available, start the service with:

```bash
WORKSTATION_ID=station-01 docker compose up -d
```

Compose requires a nonempty `WORKSTATION_ID`. No profile flag is needed.

## ROS 2 package reference

### Gazebo models

Each ROS 2 package **owns its Gazebo models in its own source tree** and
installs them with a normal `install(DIRECTORY ...)` rule. A shared CMake macro,
`gz_register_models()`, then registers that already-installed directory onto
`GZ_SIM_RESOURCE_PATH` so Gazebo can resolve `model://...` URIs. Models are
**never copied to a central location** and **never installed twice** — the macro
only wires up a runtime environment hook, it does not install anything itself.

#### The shared macro

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

#### Using it in a package

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

#### Runtime flow

1. `colcon build --symlink-install` — `install()` places models under
   `install/<pkg>/share/<pkg>/...`; the macro drops the env hook into the
   package's install prefix.
2. `source install/setup.bash` — each package's hook prepends its model dir to
   `GZ_SIM_RESOURCE_PATH`.
3. Gazebo resolves `model://basler`, `model://station`, etc. across all
   packages.
