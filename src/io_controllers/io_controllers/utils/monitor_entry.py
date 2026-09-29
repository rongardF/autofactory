# Copyright (c) 2026, Movement Controller Contributors
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
"""Per-goal watch state for the ``monitor_io`` action.

One :class:`MonitorEntry` tracks a single active ``monitor_io`` goal. It owns a
non-blocking queue that the driver change callback pushes into (enqueue-only);
the goal's own ``execute_callback`` drains it on an executor thread and publishes
feedback there, so no raw thread ever calls into ``rclpy`` (SPEC 7.5, Option B).
"""

import queue

from io_controllers.models.io_signal_dto import AnalogSignalDTO, DigitalSignalDTO

# One in-memory signal snapshot as delivered to a monitor's per-goal queue,
# paired with the ``builtin_interfaces/Time`` stamp of the observation.
ChangeEvent = tuple[DigitalSignalDTO | AnalogSignalDTO, object]


class MonitorEntry:
    """Per-goal watch state for one active ``monitor_io`` goal."""

    def __init__(self, goal_handle: object, io_name: str, is_digital: bool) -> None:
        """Initialize a watch entry for one accepted goal.

        Args:
            goal_handle: The accepted action goal handle this entry serves.
            io_name: The mapped signal name being watched.
            is_digital: True if the watched signal is digital, else analog.
        """
        self.goal_handle = goal_handle
        self.io_name = io_name
        self.is_digital = is_digital
        self.change_count = 0
        self.pending: queue.SimpleQueue = queue.SimpleQueue()
