# ROS 2 Jazzy — Patterns, Conventions & Pitfalls

## Official Documentation

**Always consult before generating ROS 2 code:**
- ROS 2 Jazzy docs: https://docs.ros.org/en/jazzy/index.html
- MoveIt2 docs: https://moveit.picknik.ai/main/index.html
- UR Robot Driver (ROS 2): https://docs.universal-robots.com/Universal_Robots_ROS2_Driver/

Do not guess topic names, service signatures, message types, or launch arguments — look them up
in the official docs for this stack.

---

## Node Lifecycle Pattern

All nodes that manage hardware, connections, or stateful services **must** use `LifecycleNode`:

```python
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

class MyHardwareNode(LifecycleNode):
    def __init__(self, node_name: str = 'my_hardware_node') -> None:
        super().__init__(node_name)
        # declare all parameters here — before on_configure
        self.declare_parameter(
            'device_ip',
            '192.168.1.100',
            ParameterDescriptor(description='IP address of the hardware device')
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # create publishers, subscribers, clients — no hardware I/O yet
        self._pub = self.create_publisher(String, '/my/topic', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # open hardware connections, start timers
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # stop timers, stop hardware commands
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # destroy publishers/subscribers/clients
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # close hardware connections
        return TransitionCallbackReturn.SUCCESS
```

**Rules:**
- Hardware I/O only happens in `on_activate` / `on_deactivate`, never in `__init__` or `on_configure`
- `on_configure` creates ROS 2 entities (pubs, subs, clients, services); `on_activate` starts them
- Always return `TransitionCallbackReturn.FAILURE` (not raise) if a transition cannot complete

---

## Action Client Pattern (Non-Blocking)

**Always** use the async + callback pattern. Never use the blocking `send_goal()`:

```python
from __future__ import annotations
from rclpy.action.client import ClientGoalHandle, CancelGoal, ActionClient
from rclpy.task import Future
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped

class RobotServiceNode(LifecycleNode):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        return TransitionCallbackReturn.SUCCESS

    def send_move(self, target_pose: PoseStamped) -> bool:
        goal = self._generate_move_goal(target_pose)
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            err = 'MoveGroup action server not available'
            self.get_logger().error(err)
            raise DependencyFailure(err)
        send_future: Future = self._move_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_accepted_callback)

        return True

    def abort_move(self) -> bool:
        if self._goal_handle is not None:
            cancel_future: Future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)

    def _goal_accepted_callback(self, future: Future[ClientGoalHandle]) -> None:
        try:
            self._goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                # perform some optional actions upon goal being rejected
                return
            result_future: Future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
        except Exception as err:
            self._logger.error(f"GOAL ACCEPTED CALLBACK ERROR: {err}")
            # handle error somehow

    def _cancel_done_callback(
        self, future: Future[CancelGoal.Response]
    ) -> None:
        # handle goal cancelling
        pass

    def _feedback_callback(self, feedback_msg: MoveGroup_FeedbackMessage) -> None:
        # handle incremental feedback
        pass

    def _result_callback(self, future: Future[MoveGroup_GetResult_Response]) -> None:
        result = future.result().result
        # handle result
        pass
```

**Never do:**
```python
# WRONG — blocks the executor thread
result = self._move_client.send_goal(goal)

# WRONG — blocks forever if server never comes up
self._move_client.wait_for_server()  # no timeout
```

---

## Parameter Declaration

Always declare parameters explicitly with a default value and a `ParameterDescriptor(description=...)` before reading them:

```python
from rcl_interfaces.msg import ParameterDescriptor

def __init__(self, node_name: str = 'my_node') -> None:
    super().__init__(node_name)
    # CORRECT — declare with default value, type, and description
    self.declare_parameter('robot_ip', '192.168.1.100', ParameterDescriptor(description='IP address of the UR10e robot controller'))
    self.declare_parameter('max_speed', 0.5, ParameterDescriptor(description='Maximum TCP speed in m/s; must not exceed the hardware safety limit'))
    self.declare_parameter('use_sim_time', False, ParameterDescriptor(description='Use ROS 2 simulation time instead of wall clock'))

def on_configure(self, state: State) -> TransitionCallbackReturn:
    robot_ip: str = self.get_parameter('robot_ip').get_parameter_value().string_value
    max_speed: float = self.get_parameter('max_speed').get_parameter_value().double_value
```

**Never do:**
```python
# WRONG — will raise if parameter not declared
robot_ip = self.get_parameter('robot_ip').value

# ALSO WRONG — missing description
self.declare_parameter('robot_ip', '192.168.1.100')
```

---

## Topic / Service / Action Naming Conventions

All robot-namespaced interfaces follow this pattern:

| Type | Pattern | Example |
|------|---------|---------|
| Robot status topics | `/my_namespace/robot/<entity>` | `/my_namespace/robot/state`, `/my_namespace/robot/tcp_pose` |

Use `snake_case` for topic/service/action names. Use `PascalCase` for message/service/action type names.

---

## Executor and Threading

- Use `MultiThreadedExecutor` when a node contains callbacks that may block or when mixing sync ROS 2 callbacks with async I/O:

```python
from rclpy import init, shutdown
from rclpy.executors import MultiThreadedExecutor

def main() -> None:
    init()
    node = MyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        shutdown()
```

- Each component that uses `asyncio` (EventHub publisher, MongoDB client) **owns its own event loop** running in a dedicated thread — never share loops between threads or mix with `rclpy` spin
- Use `asyncio.run_coroutine_threadsafe(coro, loop)` to submit work from a ROS 2 callback into an asyncio loop
- if there is a concern that some callbacks might be too long running and block some other critical callbacks then use `ReentrantCallbackGroup` callback groups

---

## Launch File Conventions

- All launch files are Python (`.launch.py`), never XML or YAML
- All hardware nodes accept `use_sim_time` as a launch argument:

```python
use_sim_time = LaunchConfiguration('use_sim_time', default='false')

Node(
    package='movement_controller',
    executable='ur_movement_controller',
    parameters=[{'use_sim_time': use_sim_time}]
)
```

- Simulation vs real hardware is determined at launch time, not in node code

---

---

## MoveIt2 Python API — moveit_py

Use `moveit_py` (the `moveit` Python package) for all motion planning. This is the **only** supported
MoveIt2 Python API in ROS 2. `MoveIt Commander` (`moveit_commander`) was ROS 1 only — do NOT use it.

### Installation

`moveit_py` is installed with `ros-jazzy-moveit` or `ros-jazzy-moveit-py`. In the devcontainer:

```bash
apt-get install ros-jazzy-moveit
```

---

## Known Jazzy Pitfalls

1. **`QoS` incompatibilities** — `/tf` and `/tf_static` require `StaticBroadcasterQoS` / `TFBroadcasterQoS`. Mismatched QoS silently drops messages; always match publisher and subscriber QoS profiles.

2. **`send_goal_async` race on fast goals** — The `wait_for_server()` call can block if the action server isn't up yet; always pass `timeout_sec` and handle the `False` return.

3. **Parameter typing in Jazzy** — `declare_parameter('key', value)` infers type from the default. If you later set a string where an int is expected, it raises. Always declare with the correct Python type.

4. **`colcon build` isolation** — Always build with `--symlink-install` in development (so Python file edits are picked up without rebuild), but CI should use a clean build without symlinks to catch missing install rules.

5. **`rclpy` logging vs Python logging** — Use `self.get_logger()` for all ROS 2 node logs (they go to `/rosout`). Use Python `logging` only for non-node utility modules. Never mix them in the same module that has a node handle.

6. **Workspace source order** — In the devcontainer, always source in this order:
   ```bash
   source /opt/ros/jazzy/setup.bash   # ROS 2 base install
   source install/setup.bash          # local workspace overlay — adds your packages to the path
   ```
   Sourcing only the base install means your packages won't be found.

7. **`package.xml` test dependencies** — For `colcon test` to discover `pytest`-based tests, `package.xml` must declare:
   ```xml
   <test_depend>ament_pytest</test_depend>
   <test_depend>pytest</test_depend>
   ```
   Without these, `colcon test` will silently skip the test phase.

8. **`from __future__ import annotations` + Pydantic v2** — This import enables PEP 563 lazy annotation evaluation. Pydantic v2 handles this correctly for pure Python types. However, models with `arbitrary_types_allowed=True` that store ROS message objects (e.g. `PoseStamped`) as fields should avoid forward-reference strings for those fields — store them as `Optional` with `model_config = ConfigDict(arbitrary_types_allowed=True)`.

9. **`moveit_py` and `rclpy.init()`** — `MoveItPy` must be instantiated **after** `rclpy.init()` is called. If using it alongside LifecycleNodes run on a MultiThreadedExecutor, instantiate `MoveItPy` in the node's `on_configure` or before spinning the executor.
