"""Shared fixtures: a spun ROS 2 context and helpers for waiting on conditions."""

import time
from collections.abc import Callable
from threading import Thread

import rclpy
from pytest import fixture
from rclpy.executors import MultiThreadedExecutor


def wait_until(condition: Callable[[], bool], timeout_s: float) -> bool:
    """Poll ``condition`` until it holds or ``timeout_s`` passes."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


@fixture(scope='session', autouse=True)
def ros_context():
    """ROS 2 context for the whole session."""
    rclpy.init()
    yield
    rclpy.shutdown()


@fixture
def executor():
    """A MultiThreadedExecutor spinning in the background; nodes are added by the tests."""
    executor = MultiThreadedExecutor(num_threads=4)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    yield executor
    for node in list(executor.get_nodes()):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
