# Copyright (c) 2026, Tools Manager Contributors
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
"""ROS 2 launch-file process manager.

``Ros2Launcher`` starts ROS 2 launch files exactly as the ``ros2 launch`` CLI
would — each launch runs in its own child process (and its own process group),
so all nodes it brings up live outside this node's process. Shutting a launch
down mirrors pressing ``CTRL+C`` in the terminal where ``ros2 launch`` was
running: the child process group is sent ``SIGINT`` so ``ros2 launch`` can tear
its nodes down gracefully, escalating to ``SIGTERM``/``SIGKILL`` if it refuses
to exit.

Each running launch is tracked in an internal registry keyed by ``launch_id``. A
per-launch daemon monitor thread watches the child process and flips the
associated :class:`LaunchDto`'s ``running`` flag to ``False`` the moment the
process exits for any reason (clean exit, crash, or a requested shutdown), so the
``LaunchDto`` returned by :meth:`launch` always reflects the live process state.
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
from uuid import UUID, uuid4

from rclpy.node import Node

from tools_manager.model.launch_dto import LaunchDto


class _LaunchProcess:
    """Internal book-keeping for a single running launch process."""

    def __init__(self, process: subprocess.Popen, dto: LaunchDto) -> None:
        self.process = process
        self.dto = dto
        self.monitor: threading.Thread | None = None


class Ros2Launcher:
    """Launch and manage ROS 2 launch files as independent child processes."""

    def __init__(
        self,
        node: Node,
        startup_grace_sec: float = 1.0,
        shutdown_timeout_sec: float = 10.0,
    ) -> None:
        """Create the launcher.

        :param node: Node used only to access the logger; the launcher does not
            create any ROS entities on it.
        :param startup_grace_sec: How long :meth:`launch` waits after spawning to
            catch an immediate failure (bad path, launch-file error) before
            reporting the launch as running.
        :param shutdown_timeout_sec: How long :meth:`shutdown` waits after each
            escalating signal (``SIGINT`` then ``SIGTERM``) before escalating
            further, ultimately to ``SIGKILL``.
        """
        self._node = node
        self._logger = node.get_logger()
        self._startup_grace_sec = startup_grace_sec
        self._shutdown_timeout_sec = shutdown_timeout_sec

        self._lock = threading.Lock()
        self._launches: dict[UUID, _LaunchProcess] = {}

    def launch(
        self,
        launch_file: str,
        launch_id: UUID | None = None,
        launch_arguments: dict[str, str | float | bool | list] | None = None,
    ) -> LaunchDto:
        """Start a ROS 2 launch file in its own child process.

        Builds and runs ``ros2 launch <launch_file> key:=value ...`` without a
        shell (argument-injection safe), in its own process group so the whole
        tree can be signalled on shutdown. Waits ``startup_grace_sec`` to catch
        an immediate failure; on failure a :class:`LaunchDto` with
        ``running=False`` is returned and nothing is registered.

        :param launch_file: Filesystem path to the launch file to run.
        :param launch_id: Optional identifier for the launch; generated when omitted.
        :param launch_arguments: Optional launch arguments passed as ``key:=value``.
        :return: A :class:`LaunchDto` describing the launch; ``running`` reflects
            whether the process is alive after the startup grace period.
        """
        arguments = dict(launch_arguments or {})
        resolved_id = launch_id if launch_id is not None else uuid4()

        argv = ['ros2', 'launch', launch_file]
        try:
            argv.extend(
                self._format_argument(key, value) for key, value in arguments.items()
            )
        except TypeError as error:
            self._logger.error(
                f'Invalid launch arguments for {launch_file!r}: {error}'
            )
            return LaunchDto(
                launch_file=launch_file,
                launch_id=resolved_id,
                launch_arguments=arguments,
                running=False,
            )

        try:
            process = subprocess.Popen(argv, start_new_session=True)
        except OSError as error:
            self._logger.error(f'Failed to start launch process for {launch_file!r}: {error}')
            return LaunchDto(
                launch_file=launch_file,
                launch_id=resolved_id,
                launch_arguments=arguments,
                running=False,
            )

        # Grace period: if the process dies immediately the launch failed to start.
        try:
            process.wait(timeout=self._startup_grace_sec)
            self._logger.error(
                f'Launch process for {launch_file!r} exited immediately '
                f'with code {process.returncode}'
            )
            return LaunchDto(
                launch_file=launch_file,
                launch_id=resolved_id,
                launch_arguments=arguments,
                running=False,
            )
        except subprocess.TimeoutExpired:
            pass

        dto = LaunchDto(
            launch_file=launch_file,
            launch_id=resolved_id,
            launch_arguments=arguments,
            running=True,
        )
        entry = _LaunchProcess(process, dto)
        with self._lock:
            self._launches[dto.launch_id] = entry

        monitor = threading.Thread(
            target=self._monitor_process,
            args=(entry,),
            name=f'ros2-launch-monitor-{dto.launch_id}',
            daemon=True,
        )
        entry.monitor = monitor
        monitor.start()

        self._logger.info(
            f'Started launch {dto.launch_id} for {launch_file!r} (pid {process.pid})'
        )
        return dto

    def shutdown(self, launch: LaunchDto) -> bool:
        """Shut down a launch and all of its nodes, like ``CTRL+C`` in the CLI.

        Sends ``SIGINT`` to the launch's process group so ``ros2 launch`` can tear
        its nodes down gracefully, escalating to ``SIGTERM`` and finally
        ``SIGKILL`` if the process does not exit within ``shutdown_timeout_sec``.

        :param launch: The launch to shut down, as returned by :meth:`launch`.
        :return: ``True`` if the process is confirmed stopped, ``False`` otherwise.
        """
        with self._lock:
            entry = self._launches.get(launch.launch_id)

        if entry is None:
            # Never registered (already failed/never started) or already reaped.
            launch.running = False
            return True

        process = entry.process
        if process.poll() is not None:
            self._logger.info(f'Launch {launch.launch_id} already stopped')
            launch.running = False
            return True

        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            self._logger.info(f'Launch {launch.launch_id} process already gone')
            launch.running = False
            return True

        for sig in (signal.SIGINT, signal.SIGTERM):
            self._logger.info(
                f'Sending {signal.Signals(sig).name} to launch {launch.launch_id} '
                f'(pgid {pgid})'
            )
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                launch.running = False
                return True
            try:
                process.wait(timeout=self._shutdown_timeout_sec)
                self._logger.info(f'Launch {launch.launch_id} stopped')
                launch.running = False
                return True
            except subprocess.TimeoutExpired:
                self._logger.warning(
                    f'Launch {launch.launch_id} did not stop after '
                    f'{signal.Signals(sig).name}; escalating'
                )

        self._logger.error(
            f'Launch {launch.launch_id} did not respond to SIGINT/SIGTERM; sending SIGKILL'
        )
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            launch.running = False
            return True
        try:
            process.wait(timeout=self._shutdown_timeout_sec)
            launch.running = False
            return True
        except subprocess.TimeoutExpired:
            self._logger.error(f'Launch {launch.launch_id} could not be killed')
            return False

    def shutdown_all(self) -> None:
        """Shut down every tracked launch.

        Intended to be called from the owning node's teardown so no launch
        processes are left orphaned when the node stops.
        """
        with self._lock:
            entries = list(self._launches.values())

        for entry in entries:
            self.shutdown(entry.dto)

    def _monitor_process(self, entry: _LaunchProcess) -> None:
        """Block until the launch process exits, then mark it not running."""
        entry.process.wait()
        entry.dto.running = False
        with self._lock:
            self._launches.pop(entry.dto.launch_id, None)
        self._logger.info(
            f'Launch {entry.dto.launch_id} exited with code {entry.process.returncode}'
        )

    @staticmethod
    def _format_argument(key: str, value: str | float | bool | list) -> str:
        """Serialize a launch argument into ``key:=value`` form for the CLI.

        List values may only contain ``str``, ``bool`` or ``float`` elements;
        anything else raises :class:`TypeError`. Booleans are serialized as
        ``true``/``false`` both as scalars and as list elements.

        :param key: Launch-argument name.
        :param value: Scalar or list argument value to serialize.
        :return: The ``key:=value`` string to pass to ``ros2 launch``.
        :raises TypeError: If a list element is not a ``str``, ``bool`` or ``float``.
        """
        if isinstance(value, list):
            elements = []
            for item in value:
                if not isinstance(item, (str, bool, float)):
                    raise TypeError(
                        f'Launch argument {key!r} list elements must be str, bool '
                        f'or float, got {type(item).__name__}'
                    )
                elements.append(Ros2Launcher._serialize_scalar(item))
            serialized = '[' + ','.join(elements) + ']'
        else:
            serialized = Ros2Launcher._serialize_scalar(value)
        return f'{key}:={serialized}'

    @staticmethod
    def _serialize_scalar(value: str | float | bool) -> str:
        """Serialize a single scalar launch value, mapping ``bool`` to ``true``/``false``."""
        # ``bool`` must be checked before ``float``/``int`` since it subclasses int.
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)