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
"""Interface to Gazebo's gz-transport services; the seam faked in unit tests."""

from abc import ABC, abstractmethod
from typing import Callable

from world_manager.models import PoseDTO


class GzTransportAdapter(ABC):
    """Thin wrapper over a gz-transport node.

    Request methods return True only when the transport answered and Gazebo accepted the request
    (``ok and reply.data``). Callbacks passed to ``subscribe_state`` run on gz threads.
    """

    @abstractmethod
    def create_model(self, world: str, model_id: str, sdf: str, pose: PoseDTO, timeout_sec: float) -> bool:
        """Call ``/world/<w>/create/blocking``."""

    @abstractmethod
    def remove_model(self, world: str, model_id: str, timeout_sec: float) -> bool:
        """Call ``/world/<w>/remove/blocking``."""

    @abstractmethod
    def set_model_pose(self, world: str, model_id: str, pose: PoseDTO, timeout_sec: float) -> bool:
        """Call ``/world/<w>/set_pose/blocking``."""

    @abstractmethod
    def list_models(self, world: str, timeout_sec: float) -> list[str] | None:
        """Names of the models in ``/world/<w>/scene/info``, or None if the service did not answer."""

    @abstractmethod
    def subscribe_state(self, topic: str, callback: Callable[[str], None]) -> bool:
        """Subscribe to a joint state topic (std StringMsg); returns False if the subscription failed."""

    @abstractmethod
    def unsubscribe(self, topic: str) -> None:
        """Drop the subscription of ``topic``."""

    @abstractmethod
    def publish_empty(self, topic: str) -> None:
        """Publish one Empty message on ``topic``."""
