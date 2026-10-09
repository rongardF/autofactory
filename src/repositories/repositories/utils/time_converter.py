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
"""Conversions between ``builtin_interfaces/Time`` and ``datetime``.

Cache entries carry an absolute expiry instant on the wire as a
``builtin_interfaces/Time`` (seconds + nanoseconds since an epoch defined by
the active clock). Internally the node compares expiry against ``now`` taken
from the node clock, so both sides use the same timeline and ``use_sim_time``
is honored automatically. These helpers translate between the ROS ``Time``
message and a timezone-aware UTC ``datetime`` for the internal DTO.
"""

from datetime import datetime, timezone

from builtin_interfaces.msg import Time

_NANOSECONDS_PER_SECOND = 1_000_000_000


def time_msg_to_datetime(time_msg: Time) -> datetime:
    """Convert a ``builtin_interfaces/Time`` message to a UTC ``datetime``.

    Args:
        time_msg: The ROS time message (``sec`` + ``nanosec``) interpreted as
            seconds since the clock epoch.

    Returns:
        datetime: A timezone-aware UTC ``datetime`` representing the instant.
    """
    total_seconds = time_msg.sec + time_msg.nanosec / _NANOSECONDS_PER_SECOND
    return datetime.fromtimestamp(total_seconds, tz=timezone.utc)


def datetime_to_time_msg(moment: datetime) -> Time:
    """Convert a ``datetime`` to a ``builtin_interfaces/Time`` message.

    Naive datetimes are assumed to be UTC. The instant is expressed as whole
    seconds plus a non-negative nanosecond remainder since the epoch.

    Args:
        moment: The instant to convert.

    Returns:
        Time: The equivalent ROS time message.
    """
    if moment.tzinfo is None:
        moment = moment.replace(tzinfo=timezone.utc)
    total_nanoseconds = round(moment.timestamp() * _NANOSECONDS_PER_SECOND)
    msg = Time()
    msg.sec = int(total_nanoseconds // _NANOSECONDS_PER_SECOND)
    msg.nanosec = int(total_nanoseconds % _NANOSECONDS_PER_SECOND)
    return msg
