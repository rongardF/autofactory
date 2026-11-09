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
"""Simulation world manager interface."""

from abc import ABC, abstractmethod

from geometry_msgs.msg import Pose


class WorldManager(ABC):
    """Spawn and remove models in a simulation world and remember the spawned ones.

    The record of spawned models outlives activation cycles so models left in
    the world can still be removed after re-activation.
    """

    @property
    @abstractmethod
    def spawned_models(self) -> tuple[str, ...]:
        """Return the names of the spawned models in spawn order."""

    @abstractmethod
    def spawn_model(self, name: str, sdf: str, pose: Pose) -> None:
        """Spawn a model at a world pose and record it.

        :param name: Unique model name in the world.
        :param sdf: SDF document of the model.
        :param pose: Pose of the model frame in the world frame.
        :raises WorldManagerError: If the model could not be spawned.
        """

    @abstractmethod
    def remove_model(self, name: str) -> None:
        """Remove a model from the world and forget it.

        :param name: Model name.
        :raises WorldManagerError: If the model could not be removed.
        """
