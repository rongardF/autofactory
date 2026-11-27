# station_utils

Shared code for all Autofactory ROS 2 packages.

| Module | Purpose |
|---|---|
| `station_utils.autofactory_base_node.AutofactoryBaseNode` | Base class of every lifecycle node; publishes failed transitions on `/transition_error` |
| `station_utils.services.node_manager.NodeManager` | Drive / observe one other lifecycle node (transitions, parameters, state callbacks, transition errors) |
| `station_utils.services.ros2_launch.Ros2Launch` | Run, monitor and stop one launch file as a child process |
| `station_utils/msg/TransitionError.msg` | Reason a node's transition failed |
| `station_utils/msg/Fault.msg` | Reserved for future use |

## AutofactoryBaseNode

```python
from rclpy.lifecycle import State, TransitionCallbackReturn
from station_utils.autofactory_base_node import AutofactoryBaseNode


class MyNode(AutofactoryBaseNode):
    def __init__(self, node_name: str = 'my_node') -> None:
        super().__init__(node_name)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if not self._connect():
            self.get_logger().error('Camera not reachable')
            self._publish_transition_error('Camera not reachable')
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS
```

- Call `_publish_transition_error(reason)` once wherever a transition returns `FAILURE`.
  Unhandled exceptions and `ERROR` returns are logged and published automatically.
- The publisher disappears when the process exits; after a SIGKILL, DDS
  removes it only after its liveliness lease (~20 s with Fast DDS).
- The topic is volatile: an error published within ~1–2 s of the publishing process
  starting (before DDS discovery completes) can be missed by `NodeManager`s.

## NodeManager

```python
from station_utils.enums import LifecycleStateEnum
from station_utils.services.node_manager import NodeManager

manager = NodeManager(self, 'movement_controller')
result = manager.configure()            # NodeManagerResultDTO(success, error_message)
if not result.success:
    # e.g. "Transition 'configure' of /movement_controller failed: <its TransitionError>"
    self.get_logger().error(result.error_message)

manager.monitor_transition_errors(True)  # collect this node's errors (off by default)
manager.transition_errors                # newest first, up to transition_errors_maxlen

handle = manager.register_state_callback(self._on_state)   # called with LifecycleStateEnum
manager.unregister_state_callback(handle)
manager.destroy()
```

- One instance per managed node; the owning node must be spun by a `MultiThreadedExecutor`.
- Calls block the calling thread (wall-clock bounded) and never raise; pass a
  `threading.Event` as `cancel_event` to abort a wait (`error_message == 'cancelled'`).
- State callbacks receive the primary lifecycle states and `OFFLINE` (node not in the graph).
- A failed transition waits up to `transition_error_wait_s` (default 3 s) for the managed
  node's `TransitionError` and puts its text into `error_message`.

## Ros2Launch

```python
from station_utils.services.ros2_launch import Ros2Launch

launch = Ros2Launch(self.get_logger(), '/abs/path/to/file.launch.py', {'use_sim_time': 'true'})
launch.launch(completed_callback=self._on_launch_ended)
launch.status      # LaunchStateEnum.NOT_LAUNCHED / RUNNING / STOPPED
launch.stop()      # SIGINT, after 10 s SIGTERM, then SIGKILL; stop(0) kills at once
launch.exit_code
```

- The launch runs in its own process group and receives SIGINT when the owning process dies.
- Its output is printed with the owning node's output (screen and/or the outer `launch.log`,
  depending on the owning node's `output=` setting).

## Tests

```bash
python -m pytest src/station_utils/tests/unit -q          # from the workspace root
python -m pytest src/station_utils/tests/integration -q   # spawns ROS 2 processes, ~40 s
```
