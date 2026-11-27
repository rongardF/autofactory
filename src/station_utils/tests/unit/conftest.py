"""Shared fixtures: a ROS 2 context for tests that create real (unspun) nodes."""

import rclpy
from pytest import fixture


@fixture(scope='session', autouse=True)
def ros_context():
    """Minimal ROS 2 context; nodes are created but never spun."""
    rclpy.init()
    yield
    rclpy.shutdown()
