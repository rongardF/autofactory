# Copyright (c) 2026, Autofactory
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Run a ROS 2 launch file as a managed child process."""

from __future__ import annotations

import ctypes
import os
import signal
import subprocess
from collections.abc import Callable
from threading import Event, Lock, Thread

from rclpy.impl.rcutils_logger import RcutilsLogger

from station_utils.enums.launch_state_enum import LaunchStateEnum

_PR_SET_PDEATHSIG = 1
_ESCALATION_TIMEOUT_S = 5.0

# Resolved before any fork: the child may only call async-signal-safe code before exec.
_prctl = ctypes.CDLL(None, use_errno=True).prctl


def _shutdown_with_parent() -> None:
    """Child pre-exec hook: receive SIGINT (graceful launch shutdown) when the parent dies."""
    _prctl(_PR_SET_PDEATHSIG, signal.SIGINT, 0, 0, 0)


class Ros2Launch:
    """Launch, monitor and stop one ROS 2 launch file.

    The ``ros2 launch`` process runs in its own process group (so it and all its
    nodes can be signalled together) and is told to shut down gracefully when
    the owning process dies. Its stdout/stderr are inherited, so its output
    appears wherever the owning node's output goes.
    """

    def __init__(
        self,
        logger: RcutilsLogger,
        launch_file_path: str,
        launch_arguments: dict[str, str] | None = None,
    ) -> None:
        """Store the launch configuration; nothing is started yet.

        :param logger: Logger used for all messages.
        :param launch_file_path: Absolute path of the launch file.
        :param launch_arguments: Launch arguments passed as ``name:=value``.
        :raises ValueError: If ``launch_file_path`` is not absolute.
        """
        if not os.path.isabs(launch_file_path):
            raise ValueError(f'Launch file path must be absolute: {launch_file_path!r}')
        self._logger = logger
        self._launch_file_path = launch_file_path
        self._launch_arguments = dict(launch_arguments or {})
        self._lock = Lock()
        self._stop_lock = Lock()
        self._status = LaunchStateEnum.NOT_LAUNCHED
        self._exit_code: int | None = None
        self._process: subprocess.Popen | None = None
        self._exited = Event()
        self._exited.set()

    @property
    def status(self) -> LaunchStateEnum:
        """Current state of the launch process."""
        with self._lock:
            return self._status

    @property
    def exit_code(self) -> int | None:
        """Return code of the last launch process; None until it has ended."""
        with self._lock:
            return self._exit_code

    def launch(self, completed_callback: Callable[[], None] | None = None) -> bool:
        """Start ``ros2 launch`` for the launch file.

        :param completed_callback: Called without arguments (from the monitor thread)
            once the process has ended, including after :meth:`stop`.
        :returns: True if the process was started; False if it is already running or
            could not be started.
        """
        with self._lock:
            if self._status == LaunchStateEnum.RUNNING:
                self._logger.warning(f'{self._launch_file_path} is already running')
                return False
            if not os.path.isfile(self._launch_file_path):
                self._logger.error(f'Launch file not found: {self._launch_file_path}')
                return False
            started = Event()
            spawn_error: list[BaseException] = []
            self._exited.clear()
            # The process is created and awaited by the same thread: PR_SET_PDEATHSIG fires
            # when the forking *thread* exits, and this thread outlives the child.
            Thread(
                target=self._run,
                args=(started, spawn_error, completed_callback),
                name=f'ros2_launch:{os.path.basename(self._launch_file_path)}',
                daemon=True,
            ).start()
            started.wait()
            if spawn_error:
                self._exited.set()
                self._logger.error(f'Failed to launch {self._launch_file_path}: {spawn_error[0]}')
                return False
            self._status = LaunchStateEnum.RUNNING
            self._exit_code = None
            self._logger.info(f'Launched {self._launch_file_path} (pid {self._process.pid})')
            return True

    def stop(self, timeout_s: float = 10.0) -> bool:
        """Stop the launch process and all processes it started.

        :param timeout_s: > 0: send SIGINT and wait up to ``timeout_s`` seconds, then
            escalate to SIGTERM and SIGKILL. 0: send SIGKILL immediately.
        :returns: True if the process is gone (also when it was not running).
        """
        with self._stop_lock:
            with self._lock:
                process = self._process if self._status == LaunchStateEnum.RUNNING else None
            if process is None:
                return True
            group = process.pid
            if timeout_s > 0:
                self._logger.info(f'Stopping {self._launch_file_path}')
                self._signal_group(group, signal.SIGINT)
                if not self._exited.wait(timeout_s):
                    self._logger.warning(f'{self._launch_file_path} did not stop in {timeout_s} s; terminating')
                    self._signal_group(group, signal.SIGTERM)
                    if not self._exited.wait(_ESCALATION_TIMEOUT_S):
                        self._logger.warning(f'{self._launch_file_path} did not terminate; killing')
            # Also reaps processes of the group that outlived the launch process itself.
            self._signal_group(group, signal.SIGKILL)
            stopped = self._exited.wait(_ESCALATION_TIMEOUT_S)
            if not stopped:
                self._logger.error(f'Failed to stop {self._launch_file_path}')
            return stopped

    def _run(
        self,
        started: Event,
        spawn_error: list[BaseException],
        completed_callback: Callable[[], None] | None,
    ) -> None:
        """Monitor thread: spawn the process, wait for it and report its end."""
        command = ['ros2', 'launch', self._launch_file_path]
        command += [f'{name}:={value}' for name, value in self._launch_arguments.items()]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
                preexec_fn=_shutdown_with_parent,  # noqa: PLW1509 - only calls prctl
            )
        except OSError as error:
            spawn_error.append(error)
            started.set()
            return
        self._process = process
        started.set()
        return_code = process.wait()
        with self._lock:
            self._status = LaunchStateEnum.STOPPED
            self._exit_code = return_code
        self._exited.set()
        self._logger.info(f'{self._launch_file_path} exited with code {return_code}')
        if completed_callback is not None:
            try:
                completed_callback()
            except Exception as error:  # noqa: BLE001 - a faulty callback must not kill the monitor
                self._logger.error(f'Completion callback of {self._launch_file_path} failed: {error!r}')

    @staticmethod
    def _signal_group(group: int, sig: signal.Signals) -> None:
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            pass
