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
"""Interface of the localization implementation used by the node."""

import threading
from abc import ABC, abstractmethod
from collections.abc import Callable

from rclpy.node import Node

from localization.models.localization_result_dto import (
    LocalizationRequestDTO,
    LocalizationResultDTO,
)
from station_workplace.models.workplace_state_dto import WorkplaceStateDTO


class LocalizationService(ABC):
    """Detects the product and publishes its pose; implemented by the mock and the real localizer."""

    @abstractmethod
    def initialize(self, node: Node, on_invalidated: Callable[[], None]) -> None:
        """Create the ROS interfaces the service needs.

        :param node: Node owning the interfaces.
        :param on_invalidated: Called when a finished localization is no longer valid.
        """

    @abstractmethod
    def teardown(self) -> None:
        """Destroy everything created by :meth:`initialize`; stops all TF publishing."""

    @abstractmethod
    def workplace_state(self) -> WorkplaceStateDTO | None:
        """Return the latest workplace state, or None if none was received yet."""

    @abstractmethod
    def localize(
        self,
        request: LocalizationRequestDTO,
        on_collection_done: Callable[[bool, int], None],
        cancel_event: threading.Event,
    ) -> LocalizationResultDTO:
        """Run the localization; never raises.

        :param request: What to localize.
        :param on_collection_done: Called once with ``(done, collection_time_s)`` when the
            image collection ends; no movement is commanded afterwards.
        :param cancel_event: Set to abort the localization.
        """

    @abstractmethod
    def invalidate(self) -> None:
        """Stop publishing the localized product frame."""
