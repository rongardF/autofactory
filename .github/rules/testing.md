# Testing Conventions

## Framework

- **Unit tests:** `pytest` + `ament_pytest`
- **No** `unittest.TestCase` classes — use plain pytest functions and fixtures
- **No** `launch_testing` for unit tests; `launch_testing` is reserved for full-node integration tests in later phases

---

## Test File Location

Tests live in a `tests/` folder at the **root of each Python package's source directory**:

```
src/movement_controller/
    movement_controller/              ← Python package source
        __init__.py
        ur_movement_controller.py
        fanuc_movement_controller.py
    tests/                  ← test files here
        unit/
            test_conversion.py
            ...
        integration/
            test_movement.py
            ...
    setup.py
    CMakeLists.txt
    package.xml
```

**Not** inside `movement_controller/movement_controller/tests/` — the `tests/` folder is a sibling to the Python package directory, not nested inside it.

---

## File Naming

| What | Pattern | Example |
|------|---------|---------|
| Module test | `test_<module_name>.py` | `test_robot_service.py` |
| Shared fixtures | `conftest.py` | `tests/conftest.py` |
| Integration test | `test_<feature>_integration.py` | `test_move_to_pose_integration.py` |

---

## Mocking Strategy

### What to mock

Mock **all hardware interfaces and external services that are not implemented within the package under test**:

| Dependency | Always Mock In |
|------------|---------------|
| `ur_robot_driver` / robot hardware | `movement_controller/`, all others |
| MoveIt2 move group | `movement_controller/`, all others |

### How to mock

Use `pytest-mock` (`mocker` fixture) — it wraps `unittest.mock` but integrates cleanly with pytest fixtures and auto-resets after each test:

```python
# tests/test_robot_service.py
from pytest import fixture
from movement_controller import UrMovementController
from movement_controller.action import TrajectoryExecution

@fixture
def robot_node(mocker):
    # Patch rclpy to avoid needing a live ROS 2 context
    mocker.patch('rclpy.init')
    mocker.patch('rclpy.shutdown')
    node = UrMovementController(node_name='ur10_robot')
    # Replace action client with a mock
    node._move_client = mocker.MagicMock()
    return node

def test_move_rejected_when_state_error(robot_node):
    robot_node._state = RobotStateEnum.ERROR
    # generate target_pose here for testing
    result = robot_node.send_move(target_pose=target_pose)
    assert result.success is False
    assert result.error_code == 'ESTOP_ACTIVE'
    robot_node._move_client.send_goal_async.assert_not_called()

def test_move_checks_workspace_bounds(robot_node, mocker):
    mocker.patch.object(robot_node, '_is_within_workspace', return_value=False)
    result = robot_node.send_move(target_pose=mocker.MagicMock())
    assert result.success is False
    assert result.error_code == 'OUT_OF_WORKSPACE'
```

### Async mocks

```python
async def test_event_published_on_job_complete(mocker):
    mock_producer = mocker.AsyncMock()
    mocker.patch('raw_events.eventhub_client.EventHubProducerClient', return_value=mock_producer)
    # ... test logic
    mock_producer.send_batch.assert_awaited_once()
```

---

## ROS 2 Context in Tests

Avoid requiring a live ROS 2 context (`rclpy.init()` → `rclpy.spin()`) for unit tests. Test node
logic directly by patching the ROS 2 infrastructure:

```python
@pytest.fixture(scope='session', autouse=True)
def ros_context():
    """Minimal ROS 2 context for tests that need it."""
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()
```

- Use the session-scoped fixture only in tests that genuinely need parameter reading/topic publishing
- For pure logic tests, mock `self.get_parameter(...)` directly — no context needed

---

## Test Quality Rules

- **No `pass` in test bodies** — every test must assert something
- **No `# pragma: no cover`** to skip safety-critical code paths
- **Test the failure path** — for every operation that can fail (hardware error, network timeout, bad config), there must be a test that exercises the failure path and asserts the correct `error_code`
- **One behavior per test** — keep tests focused; prefer many small tests over one large test
- **Descriptive names** — `test_move_rejected_when_estop_active` not `test_move_1`
- **Tests are described** — tests should have short descriptive docstring that explains what is tested. Example: `Test that robot rejects move goals when in ERROR state...`

---

## Running Tests

```bash
# From the ROS 2 workspace root
colcon test --packages-select raw_robot
colcon test-result --verbose

# With pytest directly (faster during development)
cd src/movement_controller
python -m pytest tests/ -v

# With coverage
python -m pytest tests/ --cov=movement_controller --cov-report=term-missing
```

---

## CI Requirements (Phase 1 target)

- All tests must pass in `colcon test` as part of CI
- `colcon build` must succeed cleanly before tests run
- Test coverage is tracked but no hard threshold is enforced in v1 (target: >70% for packages with safety-critical logic)
