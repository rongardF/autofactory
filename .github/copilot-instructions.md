# Movement controller ROS2 package — Copilot Instructions

## Project

ROS2 package to perform robot movements in a simple and extensible way, using MoveIt2 for motion planning and execution. This package will be extensible in a way that it can work with multiple robot platforms (Universal Robotics, Fanuc, etc.) by using the ROS2 robot driver that is provided by the vendor (e.g., `ur_robot_driver`). The movement controller provides a clean API for performing motion and controlling the scene (e.g., adding collision objects). 

## Tech Stack

- **Runtime:** Python 3.11+ (primary), C/C++ only if ROS 2 bindings or real-time requires it
- **ROS 2:** Jazzy — use `rclpy` for nodes, prefer standard packages (`ur_robot_driver`, MoveIt2, RViz, Gazebo)
- **Motion planning:** MoveIt2 Python bindings via `moveit_py` (`moveit` package) — **not** MoveIt Commander (that is ROS 1 only)
- **Data models:** Pydantic v2 for all schemas and data models, don't use dataclasses or plain dicts for structured data crossing boundaries
- **Container:** Docker Compose; base image `ros:jazzy-ros-base`

## Development Environment

This project is developed inside a **devcontainer** on Linux. Key facts every agent must know:

- **Container base image:** `ros:jazzy-ros-base` (Ubuntu 24.04 / ROS 2 Jazzy)
- **Workspace root inside container:** `/workspaces/autofactory` (or equivalent mount)
- **Always source in this order** before any ROS 2 command:
  ```bash
  source /opt/ros/jazzy/setup.bash          # ROS 2 base
  source install/setup.bash                 # colcon workspace overlay (after first build)
  ```
  If possible, add these to the `.bashrc` so they're automatic.
- **Build command for development:**
  ```bash
  colcon build --symlink-install            # symlink-install avoids rebuild on Python edits
  ```
- **Run tests:**
  ```bash
  colcon test --packages-select <pkg>
  colcon test-result --verbose
  # or directly with pytest during development:
  python -m pytest src/<pkg>/tests/ -v
  ```
- **RViz and Gazebo in devcontainer:** require GPU/display passthrough. Use X11 forwarding or a VNC sidecar. The `DISPLAY` env var must be set. In CI, use headless mode with `--ros-args -p use_sim_time:=true` and no visualization.
- **ROS_DOMAIN_ID:** set to an isolated value (e.g. `export ROS_DOMAIN_ID=42`) to avoid cross-talk with other ROS 2 instances on the host.
- **Python virtual environment:** the project uses a venv at `/opt/venv` created with `--system-site-packages`. This gives pip a clean install target while keeping all ROS 2 apt-installed Python packages (rclpy, etc.) visible. The entrypoint activates it before ROS 2 setup sourcing; `.bashrc` activates it in interactive shells. **Never use `pip3 install` or `--break-system-packages`** — all new Python dependencies must go into `requirements.txt` (runtime) or `requirements-dev.txt` (dev/CI tooling). The `python3-venv` apt package must be present before creating any venv (it is not in `ros:jazzy-ros-base` by default).
- **Python path:** after `colcon build --symlink-install`, sourcing `install/setup.bash` adds all workspace package paths to `PYTHONPATH` automatically — do not manually manipulate `PYTHONPATH`. The colcon overlay stacks on top of the already-active venv.

## Documentation References

When writing code for any component below, **consult the official docs at these URLs first** before generating code or making API decisions:

| Component | Official Docs URL |
|-----------|------------------|
| ROS 2 Jazzy, RViz | https://docs.ros.org/en/jazzy/index.html |
| MoveIt2 | https://moveit.picknik.ai/main/index.html |
| UR Robot Driver (ROS 2) | https://docs.universal-robots.com/Universal_Robots_ROS2_Driver/ |
| Gazebo (ROS 2) | https://gazebosim.org/docs/latest/library_howtos/ |

Do not guess API signatures, topic names, or parameter names for these packages — look them up.

## Key Conventions

### ROS 2 Node Patterns
- All ROS 2 nodes that manage hardware or services use `rclpy.lifecycle.LifecycleNode`
- Action clients use the async + callback pattern — **never** `send_goal` (blocking):
  ```python
  future = self._action_client.send_goal_async(goal)
  future.add_done_callback(self._goal_accepted_callback)
  # then in _goal_accepted_callback, get result handle and add another done_callback
  ```
- `wait_for_server()` must always use a timeout — never call it without one:
  ```python
  if not self._action_client.wait_for_server(timeout_sec=5.0):
      err = 'MoveGroup action server not available'
      self.get_logger().error(err)
      raise DependencyFailure(err)
  ```
- Non-ROS async work (MongoDB, EventHub, FastAPI) runs in a `MultiThreadedExecutor` with `asyncio` — never mix asyncio event loops across threads; each async-using component owns its loop
- Always declare parameters explicitly with `declare_parameter('name', default, ParameterDescriptor(description=...))` before use; never access `get_parameter()` without prior declaration
- Node `__init__` always accepts the node name as a typed argument with a default value: `def __init__(self, node_name: str = 'my_node') -> None:` — pass it to `super().__init__(node_name)`
- ROS2 interfaces (topics, services, actions) must be defined in `.msg`, `.srv`, `.action` files in a ROS2 package (sub-folders `msg`, `srv`, `action`) and built with `colcon` — never define ROS2 interfaces as plain Python classes or dicts. The interfaces should be defined in the same package that implements the node using them. For example, if `movement_controller` package defines a `TrajectoryExecution` action, it should be defined in `movement_controller/action/TrajectoryExecution.action` and not in a separate `movement_controller_interface` package. This means that our ROS2 packages are a mix of Python and C/C++ code (for messages) — consult ROS2 documentation for using `ament_cmake_python` and `rosidl_generate_interfaces` packages. 
- We are using BSD-3-Clause license, so all source files must include the appropriate license header (see existing files for template).
- Python code inside a ROS2 package should be organized in such  a way that we seprate Python files by purpose. For example, if we have data models that are used across multiple nodes, we can put them in a `models` sub-folder. If we have utility functions, we can put them in a `utils` sub-folder. If we have enums, we can put them in an `enums` sub-folder. This way we can keep our code organized and maintainable. The main node implementation should be in a root level of Python files (e.g. /src/movement_controller/movement_controller/ur_movement_controller.py). The ROS2 interfaces (actions, services, messages) should be defined in their respective sub-folders (`action`, `srv`, `msg`) at the root level of the package. For example, the `TrajectoryExecution` action should be defined in `movement_controller/action/TrajectoryExecution.action`. The final structure of the package should look like this:
  ```
  movement_controller/
  ├── action/
  │   └── TrajectoryExecution.action
  ├── msg/
  │   └── TrajectoryPath.msg
  ├── tests/
  │   ├── unit/
  │   │   ├── test_unit_converter.py
  |   │   └── ...
  │   ├── integration/
  │   │   ├── test_movement.py
  |   │   └── ... 
  ├── movement_controller/
  │   ├── __init__.py
  │   ├── models/
  │   │   ├── __init__.py
  │   │   └── trajectory_dto.py
  │   ├── enums/
  │   │   ├── __init__.py
  │   │   └── movement_type_enum.py
  │   ├── ur_movement_controller.py
  │   ├── fanuc_movement_controller.py
  │   ├── utils/
  │   │   ├── __init__.py
  │   │   └── units_converter.py
  │   ├── services/
  │   │   ├── __init__.py
  │   │   └── scene_repository.py
  │   └── ...
  ├── package.xml
  ├── CMakeLists.txt
  ├── setup.py
  └── ...
  ```

### MoveIt2 Python API (moveit_py)

Use the `moveit_py` binding (`from moveit.planning import MoveItPy`) — **never** MoveIt Commander (that is ROS 1).

```python
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped

# Instantiate once (it spins its own node internally)
moveit = MoveItPy(node_name='moveit_py_node')
arm = moveit.get_planning_component('ur_manipulator')  # group name from SRDF

# Plan to a pose goal
with arm.plan_and_execute():  # context manager for clean state
    arm.set_start_state_to_current_state()
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = 'base_link'
    # ... fill pose
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link='tool0')
    plan_result = arm.plan()
    if plan_result:
        robot = moveit.get_robot()
        moveit.execute(plan_result.trajectory, blocking=True, controllers=[])
```

See `.github/rules/ros2-jazzy.md` for the full `moveit_py` pattern and the correct planning group name for the UR10e.

### Error Handling
- Hardware operations and service calls return **explicit result objects** at node/package boundaries — do not let exceptions propagate across boundaries:
  ```python
  from movement_controller.action import TrajectoryExecution

  try:
      # ... perform some operation that may fail, e.g. call an action server
      result = TrajectoryExecution.Result()
      result.success = True
      return result
  except Exception as e:
      self.get_logger().error(f'Operation failed: {e}')
      result = TrajectoryExecution.Result()
      result.success = False
      result.error_message = str(e)
      return result
  ```
- Exceptions are acceptable within a single package's internal implementation; they must be caught and converted to result objects at boundaries
- Always log the error before returning a failure result so context isn't lost

### Data Models
- Pydantic v2 for all data structures
- All data model fields must have descriptions in the Pydantic model for auto-generated docs and better code readability
- If possible, data model fields should have default values to avoid missing data issues; if not, they should be required and validated by Pydantic
- Data models that are used only inside a package should have a suffix `DTO` (e.g., `TrajectoryDTO`) to distinguish them from ROS 2 message types and other classes; data models that represent ROS 2 messages (e.g., `PoseStamped`) should not have the `DTO` suffix to avoid confusion with the actual ROS 2 message classes
- Avoid using strings where enumerated values are expected; use Python `Enum` classes for better type safety and validation in Pydantic models
- Enum classes should have a `suffix` of `Enum` (e.g., `MovementTypeEnum`) and should inherit from `str` and `Enum` to be compatible with Pydantic's JSON serialization
- Data models should be frozen if not meant to be modified after creation, to prevent accidental changes and improve immutability guarantees
- Data models should be organized into separate files with each file containing only related models (e.g., `trajectory_dto.py` for trajectory-related models) to improve maintainability and readability

### Testing
- Test framework: `pytest` + `ament_pytest`
- Tests live in a `tests/` folder at the root of each Python package's source directory
- File naming: `test_<module_name>.py`
- Mock all hardware interfaces and external services (MongoDB, EventHub, robot driver) that are **not** implemented within the package under test
- `package.xml` must declare `<test_depend>ament_pytest</test_depend>` and `<test_depend>pytest</test_depend>` for `colcon test` to discover tests
- See `.github/rules/testing.md` for full testing conventions

### Simulation
- Target simulation stack is RViz (visualization) + Gazebo **Harmonic** (physics — the Gazebo version paired with ROS 2 Jazzy)
- Use `ur_robot_driver`'s `fake_hardware_interface` for robot simulation; all nodes must respect `use_sim_time` ROS 2 parameter
- Simulation/real branching is handled at the **launch layer** — individual node code does not branch on `use_sim`
- See `.github/rules/simulation.md` for full simulation conventions

## Project Rules

Detailed rules for specific domains — loaded by planning and execution agents as needed:

- `.github/rules/ros2-jazzy.md` — ROS 2 Jazzy patterns, naming conventions, pitfalls
- `.github/rules/python-patterns.md` — Result objects, async/callback patterns, type annotations
- `.github/rules/testing.md` — pytest conventions, mocking strategy, test structure
- `.github/rules/simulation.md` — Gazebo/RViz simulation stack, fake hardware, sim flags

## Project Skills

Reusable fix/task skills — load via `.github/skills/<name>/SKILL.md` when the matching scenario is detected:

- `.github/skills/fix-collada-mesh/` — Fix Gazebo segfault caused by COLLADA `.dae` meshes with VERTEX/NORMAL inputs sharing `offset="0"` in a `<polylist>` (Assimp/THREE.js exports). Use when `[gazebo-N] Segmentation fault` appears on launch and a custom `.dae` visual mesh is involved.

<!-- GSD Configuration — managed by get-shit-done installer -->
# Instructions for GSD

- Use the get-shit-done skill when the user asks for GSD or uses a `gsd-*` command.
- Treat `/gsd-...` or `gsd-...` as command invocations and load the matching file from `.github/skills/gsd-*`.
- When a command says to spawn a subagent, prefer a matching custom agent from `.github/agents`.
- Do not apply GSD workflows unless the user explicitly asks for them.
- After completing any `gsd-*` command (or any deliverable it triggers: feature, bug fix, tests, docs, etc.), ALWAYS: (1) offer the user the next step by prompting via `ask_user`; repeat this feedback loop until the user explicitly indicates they are done.
<!-- /GSD Configuration -->
