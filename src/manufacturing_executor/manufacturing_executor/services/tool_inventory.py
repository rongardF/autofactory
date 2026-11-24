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
"""Knows which tools are on the rack and in the tool mount."""

import threading
import time

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription

from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.utils.node_names import topic
from repositories.models.operation_repository.operation_dto import EndToolDTO, MaterialDTO
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.msg import Slots, ToolInfo


class ToolInventory:
    """Latest ``tool_rack/slots`` and ``tool_mount/tool_mounted`` (both latched)."""

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(
        self, node: Node, tool_rack_node_name: str, tool_mount_node_name: str, callback_group: CallbackGroup
    ) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._slots: SlotsDto | None = None
        self._mounted: ToolInfoDto | None = None
        self._subscriptions: list[Subscription] = [
            node.create_subscription(
                Slots, topic(tool_rack_node_name, 'slots'), self._on_slots, self._QOS,
                callback_group=callback_group,
            ),
            node.create_subscription(
                ToolInfo, topic(tool_mount_node_name, 'tool_mounted'), self._on_mounted, self._QOS,
                callback_group=callback_group,
            ),
        ]

    def destroy(self) -> None:
        """Destroy the subscriptions."""
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()

    def _on_slots(self, message: Slots) -> None:
        slots = SlotsDto.from_slots_msg(message)
        with self._lock:
            self._slots = slots

    def _on_mounted(self, message: ToolInfo) -> None:
        mounted = ToolInfoDto.from_msg(message)
        with self._lock:
            self._mounted = mounted

    def wait_ready(self, timeout_s: float, ctx: AbortContext) -> None:
        """Wait until both topics delivered a message.

        :raises SystemFailureError: On timeout.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            with self._lock:
                ready = self._slots is not None and self._mounted is not None
            if ready:
                return
            ctx.raise_if_aborted()
            if time.monotonic() > deadline:
                raise SystemFailureError('Tool inventory is not available (tool_rack/slots or tool_mount/tool_mounted)')
            time.sleep(0.05)

    def mounted_tool_sn(self) -> str:
        """Serial number of the mounted tool; empty if none."""
        with self._lock:
            return self._mounted.tool_sn if self._mounted is not None else ''

    def find(self, end_tool: EndToolDTO, material: MaterialDTO) -> ToolInfoDto | None:
        """Tool with exactly the given end tool and material.

        With several matches the mounted tool wins, otherwise the lowest slot index.
        """
        with self._lock:
            mounted = self._mounted
            candidates: list[ToolInfoDto] = []
            if mounted is not None and mounted.tool_sn:
                candidates.append(mounted)
            if self._slots is not None:
                candidates.extend(self._slots.tools_mounted)
        matches = [
            tool
            for tool in candidates
            if tool.tool_sn
            and tool.tool_part_number == end_tool.product_number
            and tool.tool_part_revision == end_tool.product_revision
            and tool.material_part_number == material.product_number
            and tool.material_part_revision == material.product_revision
        ]
        if not matches:
            return None
        mounted_sn = mounted.tool_sn if mounted is not None else ''
        return min(matches, key=lambda tool: (tool.tool_sn != mounted_sn, tool.index))
