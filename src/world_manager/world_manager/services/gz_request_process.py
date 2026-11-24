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
"""Blocking gz-transport service requests executed in a helper process.

The gz-transport Python binding keeps the GIL while ``Node.request`` blocks, and its subscription
callbacks acquire the GIL on the single gz-transport reception thread. A subscribed message that
arrives during a request therefore stalls the reception thread, so the reply is only seen after the
request timed out. The helper process has no subscriptions; the parent waits on a pipe without the GIL.
"""

import os
import signal
import subprocess
import sys
from multiprocessing.connection import Connection
from threading import Lock

from rclpy.impl.rcutils_logger import RcutilsLogger

_STOP_TIMEOUT_SEC = 2.0


class GzRequestProcess:
    """Sends gz-transport service requests through a lazily started helper process (thread-safe)."""

    def __init__(self, logger: RcutilsLogger, reply_margin_sec: float = 2.0) -> None:
        """Store settings; the helper is started on the first request."""
        self._logger = logger
        self._reply_margin_sec = reply_margin_sec
        self._lock = Lock()
        self._process: subprocess.Popen | None = None
        self._tx: Connection | None = None
        self._rx: Connection | None = None

    def _start(self) -> None:
        request_r, request_w = os.pipe()
        reply_r, reply_w = os.pipe()
        try:
            self._process = subprocess.Popen(
                [sys.executable, '-m', __name__, str(request_r), str(reply_w)],
                pass_fds=(request_r, reply_w), stdin=subprocess.DEVNULL)
        except Exception:
            for fd in (request_r, request_w, reply_r, reply_w):
                os.close(fd)
            raise
        os.close(request_r)
        os.close(reply_w)
        self._tx = Connection(request_w, readable=False)
        self._rx = Connection(reply_r, writable=False)

    def _stop(self) -> None:
        for connection in (self._tx, self._rx):
            if connection is not None:
                connection.close()
        self._tx = None
        self._rx = None
        if self._process is not None:
            try:
                self._process.wait(timeout=_STOP_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            self._process = None

    def request(
        self, service: str, request: bytes, request_type: str, response_type: str, timeout_ms: int,
    ) -> tuple[bool, bytes]:
        """Call ``service`` with a serialized request; returns (answered, serialized reply)."""
        with self._lock:
            if self._process is None or self._process.poll() is not None:
                self._stop()
                self._start()
            assert self._tx is not None and self._rx is not None
            try:
                self._tx.send((service, request, request_type, response_type, timeout_ms))
                if self._rx.poll(timeout_ms / 1000.0 + self._reply_margin_sec):
                    return self._rx.recv()
                self._logger.error(f'gz request helper did not return from {service}; restarting it')
            except (EOFError, OSError) as e:
                self._logger.error(f'gz request helper failed on {service}: {e}; restarting it')
            if self._process is not None:
                self._process.kill()
            self._stop()
            return False, b''

    def close(self) -> None:
        """Stop the helper process; a later request starts a new one."""
        with self._lock:
            self._stop()


def _serve(request_fd: int, reply_fd: int) -> None:
    from gz.transport13 import Node

    # Ctrl-C reaches the whole launch process group; the helper exits when the parent closes the pipe.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node = Node()
    with Connection(request_fd, writable=False) as rx, Connection(reply_fd, readable=False) as tx:
        while True:
            try:
                service, request, request_type, response_type, timeout_ms = rx.recv()
            except EOFError:
                return
            ok, reply = node.request_raw(service, request, request_type, response_type, timeout_ms)
            tx.send((bool(ok), bytes(reply)))


if __name__ == '__main__':
    _serve(int(sys.argv[1]), int(sys.argv[2]))
