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
"""Attaches and detaches DetachableJoints and confirms the result from state events."""

from functools import partial
from threading import Condition
from time import monotonic
from typing import Iterable

from rclpy.impl.rcutils_logger import RcutilsLogger

from world_manager.enums import JointStateEnum
from world_manager.exceptions import (
    ConfirmationTimeoutError,
    GzRequestError,
    ModelNotFoundError,
    ModelValidationError,
    WorldManagerError,
)
from world_manager.services.gz_transport_adapter import GzTransportAdapter
from world_manager.services.model_registry import ModelRegistry
from world_manager.utils import Deadline

class DetachableJointController:
    """Drives joints through their gz topics; state is read back from the registry.

    State handlers run on gz threads: they only update the registry and wake waiters, they never
    take the manager's operation lock and never touch rclpy objects.
    """

    def __init__(
        self,
        adapter: GzTransportAdapter,
        registry: ModelRegistry,
        republish_period_sec: float,
        logger: RcutilsLogger,
    ) -> None:
        """Store collaborators."""
        self._adapter = adapter
        self._registry = registry
        self._period = republish_period_sec
        self._logger = logger
        self._cond = Condition()

    def watch(self, topics: Iterable[str]) -> None:
        """Subscribe to joint state topics (idempotent); raises GzRequestError if one cannot be subscribed."""
        for topic in topics:
            if not self._adapter.subscribe_state(topic, partial(self._on_state, topic)):
                raise GzRequestError(f'cannot subscribe to joint state topic {topic}')

    def unwatch(self, topics: Iterable[str]) -> None:
        """Drop subscriptions that no registered or pending joint needs any more."""
        for topic in topics:
            if not self._registry.topic_in_use(topic):
                self._adapter.unsubscribe(topic)

    def _on_state(self, topic: str, data: str) -> None:
        self._registry.apply_state_event(topic, data)
        with self._cond:
            self._cond.notify_all()

    def _reached(self, model_id: str, link_name: str, desired: JointStateEnum) -> bool:
        info = self._registry.joint_state(model_id, link_name)
        return info is None or info.state == desired

    def set_attached(
        self,
        model_id: str,
        link_name: str,
        attached: bool,
        confirm_timeout_sec: float,
        deadline: Deadline | None = None,
    ) -> None:
        """Attach or detach one joint and wait for the state event.

        A joint already in the wanted state returns immediately. The request is republished every
        ``republish_period_sec`` because the plugin may not be ready for the first one.
        """
        desired = JointStateEnum.ATTACHED if attached else JointStateEnum.DETACHED
        info = self._registry.joint_state(model_id, link_name)
        if info is None:
            raise ModelNotFoundError(f"model '{model_id}' has no joint with child link '{link_name}'")
        if info.state == desired:
            return
        config = next(joint for joint, _ in self._registry.joints(model_id) if joint.link_name == link_name)
        if not info.controllable:
            topic = self._registry.collision_topic(model_id, link_name)
            raise ModelValidationError(
                f"joint '{link_name}' of '{model_id}' is not controllable: topic {topic} is shared with another joint")

        topic = config.attach_topic if attached else config.detach_topic
        verb = 'attach' if attached else 'detach'
        confirm_limit = monotonic() + confirm_timeout_sec
        while True:
            if deadline is not None:
                deadline.check(f'{verb} {model_id}/{link_name}')
            self._adapter.publish_empty(topic)
            wait = min(self._period, max(0.0, confirm_limit - monotonic()))
            if deadline is not None:
                wait = min(wait, deadline.remaining())
            with self._cond:
                confirmed = self._cond.wait_for(
                    lambda: self._reached(model_id, link_name, desired), timeout=wait)
            if confirmed:
                return
            if monotonic() >= confirm_limit:
                raise ConfirmationTimeoutError(
                    f"{verb} of '{model_id}/{link_name}' was not confirmed within {confirm_timeout_sec} s")

    def detach_all_best_effort(self, model_id: str, timeout_sec: float) -> None:
        """Detach every joint of a model before removal; failures are logged and never raised.

        gz-sim keeps a removed model's joint in the physics engine, hence the detach.
        """
        for config, info in self._registry.joints(model_id):
            if info.state == JointStateEnum.DETACHED:
                continue
            try:
                self.set_attached(model_id, config.link_name, False, timeout_sec)
            except WorldManagerError as e:
                self._logger.warning(
                    f"best-effort detach of '{model_id}/{config.link_name}' before removal failed: {e}; "
                    'the joint may stay in the physics engine')
