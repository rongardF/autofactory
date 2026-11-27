"""Integration test: /transition_error entities disappear with the node process."""

import signal
import subprocess
import sys

import rclpy
from pytest import mark

from station_utils.autofactory_base_node import TRANSITION_ERROR_TOPIC

from conftest import wait_until


_VICTIM = '''
import rclpy
from station_utils.autofactory_base_node import AutofactoryBaseNode
rclpy.init()
from station_utils.services.node_manager import NodeManager
node = AutofactoryBaseNode('victim')
manager = NodeManager(node, 'some_managed_node')
print('up', flush=True)
rclpy.spin(node)
'''

# Fast DDS removes the entities of a hard-killed participant after its 20 s lease.
_LEASE_TIMEOUT_S = 30.0


@mark.parametrize('sig, timeout_s', [(signal.SIGINT, 5.0), (signal.SIGKILL, _LEASE_TIMEOUT_S)])
def test_entities_are_removed_when_process_dies(sig, timeout_s):
    """Test that the base node's publisher and its NodeManager's subscription disappear with the process."""
    observer = rclpy.create_node('observer')
    victim = subprocess.Popen([sys.executable, '-c', _VICTIM], stdout=subprocess.PIPE, text=True)
    try:
        assert victim.stdout.readline().strip() == 'up'

        def entities() -> tuple[int, int]:
            return (observer.count_publishers(TRANSITION_ERROR_TOPIC),
                    observer.count_subscribers(TRANSITION_ERROR_TOPIC))

        assert wait_until(lambda: entities() == (1, 1), 10.0)
        victim.send_signal(sig)
        victim.wait()
        assert wait_until(lambda: entities() == (0, 0), timeout_s)
    finally:
        victim.kill()
        observer.destroy_node()
