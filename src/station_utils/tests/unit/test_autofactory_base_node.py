"""Unit tests of AutofactoryBaseNode transition-error reporting."""

from unittest.mock import MagicMock

from pytest import fixture
from rclpy.lifecycle import TransitionCallbackReturn

from station_utils.autofactory_base_node import AutofactoryBaseNode
from station_utils.msg import TransitionError


class _Node(AutofactoryBaseNode):
    def __init__(self) -> None:
        super().__init__('base_under_test', namespace='ns')
        self.configure_behavior = TransitionCallbackReturn.SUCCESS

    def on_configure(self, state):
        if isinstance(self.configure_behavior, Exception):
            raise self.configure_behavior
        return self.configure_behavior


@fixture
def node():
    node = _Node()
    node._transition_error_publisher = MagicMock()
    yield node
    node.destroy_node()


def _published(node) -> list[TransitionError]:
    return [call.args[0] for call in node._transition_error_publisher.publish.call_args_list]


def test_published_error_carries_message_and_fully_qualified_source(node):
    """Test the fields of a published transition error."""
    node._publish_transition_error('camera not found')
    (msg,) = _published(node)
    assert msg.message == 'camera not found'
    assert msg.source_node == '/ns/base_under_test'
    assert (msg.stamp.sec, msg.stamp.nanosec) != (0, 0)


def test_exception_in_transition_publishes_error_and_returns_error(node):
    """Test that an unhandled exception in a transition is reported and ends in ERROR."""
    node.configure_behavior = RuntimeError('driver crashed')
    result = node.trigger_configure()
    assert result == TransitionCallbackReturn.ERROR
    (msg,) = _published(node)
    assert 'configure' in msg.message and 'driver crashed' in msg.message


def test_error_return_publishes_generic_error(node):
    """Test that a transition returning ERROR is reported."""
    node.configure_behavior = TransitionCallbackReturn.ERROR
    node.trigger_configure()
    (msg,) = _published(node)
    assert 'configure' in msg.message


def test_failure_return_publishes_nothing_automatically(node):
    """Test that FAILURE is left to the subclass to report explicitly."""
    node.configure_behavior = TransitionCallbackReturn.FAILURE
    assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
    assert _published(node) == []
