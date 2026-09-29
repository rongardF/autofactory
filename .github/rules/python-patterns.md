# Python Patterns — Conventions for This Project


## Async Pattern — MultiThreadedExecutor + asyncio

Non-ROS async work (MongoDB via `motor`, Azure EventHub, FastAPI) runs alongside ROS 2 nodes using
a `MultiThreadedExecutor` with a dedicated `asyncio` loop per component.

### Pattern: asyncio loop in dedicated thread

```python
from asyncio import new_event_loop, AbstractEventLoop, run_coroutine_threadsafe, 
from threading import Thread
from motor.motor_asyncio import AsyncIOMotorClient
from rcl_interfaces.msg import ParameterDescriptor

class EventPublisherNode(LifecycleNode):
    def __init__(self, node_name: str = 'event_publisher') -> None:
        super().__init__(node_name)
        self.declare_parameter('mongo_uri', '', ParameterDescriptor(description='MongoDB connection URI for the event persistence store'))
        self._loop: AbstractEventLoop = new_event_loop()
        self._loop_thread = Thread(target=self._loop.run_forever, daemon=True)
        self._mongo_client: AsyncIOMotorClient | None = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._loop_thread.start()
        mongo_uri = self.get_parameter('mongo_uri').get_parameter_value().string_value
        # IMPORTANT: Motor 3.x removed the `io_loop` constructor argument.
        # Create the client inside the dedicated event loop using run_coroutine_threadsafe.
        future = run_coroutine_threadsafe(self._init_mongo(mongo_uri), self._loop)
        future.result(timeout=10.0)  # block until client initialised
        return TransitionCallbackReturn.SUCCESS

    async def _init_mongo(self, uri: str) -> None:
        # Motor 3.x: no io_loop arg — client binds to the running loop automatically
        self._mongo_client = AsyncIOMotorClient(uri)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=5.0)
        return TransitionCallbackReturn.SUCCESS

    def publish_event(self, event: WorkstationEvent) -> None:
        # called from a ROS 2 callback — submit to asyncio loop
        run_coroutine_threadsafe(
            self._publish_async(event),
            self._loop
        )

    async def _publish_async(self, event: WorkstationEvent) -> None:
        await self._eventhub_producer.send_batch([EventData(event.model_dump_json())])
```

### Rules

- Each component that uses asyncio **owns exactly one event loop** in one dedicated thread
- Never call `asyncio.run()` inside a ROS 2 callback — it creates a new loop and blocks
- Never use `await` in a function that may be called from a ROS 2 executor thread
- Use `asyncio.run_coroutine_threadsafe(coro, loop)` to bridge from a sync callback into the async world
- FastAPI + uvicorn always run in their own process or thread with their own event loop — never shared with a ROS 2 node

---

## Action Client Callback Pattern

See `.github/rules/ros2-jazzy.md` for the full pattern. Summary:

```python
# Step 1 — send and receive goal acceptance callback
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
future.add_done_callback(self._goal_accepted_callback)

# Step 2 — in acceptance callback, hook result callback
def _goal_accepted_callback(self, future: Future[ClientGoalHandle]) -> None:
    goal_handle = future.result()
    if not goal_handle.accepted:
        return  # handle rejection
    result_future: Future = goal_handle.get_result_async()
    result_future.add_done_callback(self._result_callback)

# Step 3 — handle final result
def _result_callback(self, future: Future[MoveGroup_GetResult_Response]) -> None:
    result = future.result().result
    # handle result
```

---

## Type Annotations

All functions and methods **must** have full type annotations:

```python
# CORRECT
def move_to_pose(self, pose: PoseStamped, speed_scale: float = 1.0) -> MoveResult:
    ...

def fetch_recipe(self, recipe_id: str) -> Recipe | None:
    ...

async def publish_event(self, event: WorkstationEvent) -> None:
    ...
```

- Use `from __future__ import annotations` at the top of every module to enable forward references
- Use `X | Y` union syntax (Python 3.10+), not `Optional[X]` or `Union[X, Y]`
- Use `from typing import TYPE_CHECKING` to avoid circular imports in type hints

---

## Pydantic v2 Data Models

All data structures crossing package/node/service boundaries use Pydantic v2:

```python
from pydantic import BaseModel, Field
from typing import Literal

class DispenseOperation(BaseModel):
    type: Literal['dispense'] = Field(default='dispense', description='Discriminator field identifying this as a dispensing operation')
    end_tool_id: str = Field(description='ID of the dispenser end-tool to use, as defined in the tool registry')
    material_product_number: str = Field(description='Product number of the material to dispense; must be loaded in the specified tool')
    volume_ml: float = Field(gt=0, le=1000, description='Volume to dispense in millilitres; must be > 0 and <= 1000')
    speed_mm_s: float = Field(default=50.0, gt=0, description='TCP travel speed during dispense path in mm/s')

class Recipe(BaseModel):
    id: str = Field(description='Unique recipe identifier used to fetch this recipe from MongoDB')
    name: str = Field(description='Human-readable recipe name displayed in the operator UI')
    operations: list[DispenseOperation] = Field(description='Ordered list of operations to execute; executed sequentially')

# Serialization
recipe_json = recipe.model_dump_json()
recipe = Recipe.model_validate_json(recipe_json)
```

**Rules:**
- `model_dump()` / `model_validate()` — use v2 API, never `.dict()` or `.parse_obj()`
- Every field **must** use `Field(description=...)` — descriptions are required on all model attributes
- Use `Field(...)` for validation constraints (gt, lt, min_length, pattern) in addition to the description
- Models are immutable by default — do not set `model_config = ConfigDict(frozen=False)` without justification

---

## Import Conventions

```python
# Standard library first
from __future__ import annotations
from asyncio import new_event_loop, AbstractEventLoop
from threading import Thread

# Third-party next
from rclpy.lifecycle import LifecycleNode
from pydantic import BaseModel

# Local package last
from raw_robot.result_types import MoveResult
from raw_robot.safety_monitor import SafetyMonitor
```

- Absolute imports only — no relative imports (`.something`)
- Strict import if possible, import only what is needed (`from numpy import ndarray, array`instead of `import numpy`)
- Never use wildcard imports (`from module import *`)
- Imports should be imported at the module level and **never** inside a Python class or a function. Only exception to this can be when creating test functions.

```python
# NEVER DO (NOT CORRECT)
def move_to_pose(self, pose: PoseStamped, speed_scale: float = 1.0) -> MoveResult:
    from os import path
    ...
```

## Python files

- Python files should be named using `snake_case`
- Python classes should be named using `PascalCase`
- Python files should be grouped into sub-folders with `__init__.py` file if related logic, function or purpose. This makes their importing easier and importing order can be resolved in `__init__.py` of that sub-folder (helps to avoid circular import issues). For example, if we have multiple files that define some Pydantic data models then we can place them into single sub-folder called `models` and add an `__init__.py` file. In that file we will resolve importing Python classes from each module:

```python
# __init__.py file in 'models' sub-folder
from .pose_stamped_dto import PoseStampedDTO  # must be imported first because TrajectoryPathDTO depends on it
from .trajectory_path_dto import TrajectoryPathDTO
```

Exception to this can be automated code tests (with pytest).