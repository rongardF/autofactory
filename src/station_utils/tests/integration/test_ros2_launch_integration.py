"""Integration tests of Ros2Launch with real ``ros2 launch`` processes."""

import os
import signal
import subprocess
import sys
from pathlib import Path
from threading import Event
from unittest.mock import MagicMock

from pytest import fixture, raises

from station_utils.enums import LaunchStateEnum
from station_utils.services.ros2_launch import Ros2Launch

from conftest import wait_until

LAUNCH_FILE = str(Path(__file__).parent / 'assets' / 'sleeper.launch.py')
START_TIMEOUT_S = 20.0


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A zombie is dead for our purposes.
    with open(f'/proc/{pid}/stat') as stat:
        return stat.read().split()[2] != 'Z'


def _child_pid(pid_file: Path) -> int:
    assert wait_until(lambda: pid_file.exists() and pid_file.read_text().strip(), START_TIMEOUT_S)
    return int(pid_file.read_text())


@fixture
def pid_file(tmp_path) -> Path:
    return tmp_path / 'child.pid'


@fixture
def launcher(pid_file):
    launcher = Ros2Launch(MagicMock(), LAUNCH_FILE, {'pid_file': str(pid_file)})
    yield launcher
    launcher.stop(timeout_s=0)


def test_relative_path_is_rejected():
    """Test that a relative launch file path raises ValueError."""
    with raises(ValueError):
        Ros2Launch(MagicMock(), 'sleeper.launch.py')


def test_status_is_not_launched_initially(launcher):
    """Test the initial status and exit code."""
    assert launcher.status == LaunchStateEnum.NOT_LAUNCHED
    assert launcher.exit_code is None


def test_stop_before_launch_is_noop(launcher):
    """Test that stopping a never launched instance reports success."""
    assert launcher.stop() is True


def test_missing_launch_file_is_not_launched(tmp_path):
    """Test that a non-existent launch file fails without raising."""
    launcher = Ros2Launch(MagicMock(), str(tmp_path / 'missing.launch.py'))
    assert launcher.launch() is False
    assert launcher.status == LaunchStateEnum.NOT_LAUNCHED


def test_launch_runs_launch_file_with_arguments(launcher, pid_file):
    """Test that launch starts the launch file and passes the launch arguments."""
    assert launcher.launch() is True
    assert launcher.status == LaunchStateEnum.RUNNING
    assert _alive(_child_pid(pid_file))


def test_second_launch_while_running_is_refused(launcher, pid_file):
    """Test that launch returns False while the process is running."""
    launcher.launch()
    _child_pid(pid_file)
    assert launcher.launch() is False


def test_natural_exit_sets_stopped_and_calls_callback(pid_file):
    """Test that the end of the process is detected, with exit code and callback."""
    done = Event()
    launcher = Ros2Launch(MagicMock(), LAUNCH_FILE, {'pid_file': str(pid_file), 'duration': '0'})
    launcher.launch(done.set)
    assert done.wait(START_TIMEOUT_S)
    assert launcher.status == LaunchStateEnum.STOPPED
    assert launcher.exit_code == 0


def test_graceful_stop_ends_launched_processes(launcher, pid_file):
    """Test that stop shuts down the launch process and the processes it started."""
    done = Event()
    launcher.launch(done.set)
    child = _child_pid(pid_file)
    assert launcher.stop(timeout_s=10.0) is True
    assert launcher.status == LaunchStateEnum.STOPPED
    assert not _alive(child)
    assert done.is_set()


def test_zero_timeout_kills_immediately(launcher, pid_file):
    """Test that stop(timeout_s=0) kills the whole process group."""
    launcher.launch()
    child = _child_pid(pid_file)
    assert launcher.stop(timeout_s=0) is True
    assert launcher.exit_code == -signal.SIGKILL
    assert wait_until(lambda: not _alive(child), 2.0)


def test_relaunch_after_stop(launcher, pid_file):
    """Test that a stopped instance can be launched again."""
    launcher.launch()
    _child_pid(pid_file)
    launcher.stop()
    pid_file.unlink()
    assert launcher.launch() is True
    assert _alive(_child_pid(pid_file))


_OWNER = '''
import sys, time
from unittest.mock import MagicMock
from station_utils.services.ros2_launch import Ros2Launch
Ros2Launch(MagicMock(), sys.argv[1], {'pid_file': sys.argv[2]}).launch()
print('launched', flush=True)
time.sleep(600)
'''


def test_launch_stops_when_owner_process_is_killed(pid_file):
    """Test that the launched processes shut down when the owning process dies."""
    owner = subprocess.Popen(
        [sys.executable, '-c', _OWNER, LAUNCH_FILE, str(pid_file)], stdout=subprocess.PIPE, text=True)
    try:
        assert owner.stdout.readline().strip() == 'launched'
        child = _child_pid(pid_file)
        owner.kill()
        owner.wait()
        assert wait_until(lambda: not _alive(child), 15.0)
    finally:
        owner.kill()
