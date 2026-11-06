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
"""Abstract planning-scene manager."""

import threading
from abc import ABC, abstractmethod
from collections.abc import Iterable
from pathlib import Path

from geometry_msgs.msg import Pose


class PlanningSceneManager(ABC):
    """Manage the workplace objects attached to the carrier in the planning scene.

    Every method that talks to the planning scene takes a ``cancel`` token,
    raises ``PlanningSceneError`` on failure or timeout and
    ``ProcessCancelledError`` when cancelled.
    """

    @abstractmethod
    def activate(self) -> None:
        """Create the ROS entities (service clients)."""

    @abstractmethod
    def deactivate(self) -> None:
        """Destroy the ROS entities."""

    @abstractmethod
    def get_present_object_ids(self, cancel: threading.Event) -> set[str]:
        """Return the IDs of all attached and world collision objects."""

    @abstractmethod
    def attach_safety_object(self, cancel: threading.Event) -> None:
        """Attach (or replace) the safety box on the attach link."""

    @abstractmethod
    def attach_product_and_carrier(
        self,
        product_mesh_path: Path,
        carrier_mesh_path: Path,
        product_pose: Pose,
        cancel: threading.Event,
    ) -> None:
        """Replace the safety object with the carrier and product models in one update.

        The meshes are loaded before the planning scene is touched, so a mesh
        error leaves the scene unchanged.
        """

    @abstractmethod
    def remove_objects(self, object_ids: Iterable[str], cancel: threading.Event) -> None:
        """Remove the given objects; IDs that are not present are ignored."""
