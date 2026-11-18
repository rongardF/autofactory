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
"""Declaration and reading of the tool parameters shared by the tools manager and the tool rack."""

from __future__ import annotations

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter

from tools_manager.model.simulation_inventory_dto import SimulationInventoryDTO
from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO

TOOLS_INDEX = 'tools.index'
TOOLS_TOOL_SN = 'tools.tool_sn'
TOOLS_NODE_NAME = 'tools.node_name'
SIMULATION_RACK_INDEX = 'simulation.tool_rack.index'
SIMULATION_RACK_TAG_DATA = 'simulation.tool_rack.tag_data'
SIMULATION_TOOL_MOUNT = 'simulation.tool_mount'


def declare_tools_parameters(node: Node) -> None:
    """Declare the expected tool assignment arrays (``tools.*``) on a node."""
    node.declare_parameter(
        TOOLS_INDEX,
        Parameter.Type.INTEGER_ARRAY,
        ParameterDescriptor(description='Index of every physical rack slot, including unassigned slots.'),
    )
    node.declare_parameter(
        TOOLS_TOOL_SN,
        Parameter.Type.STRING_ARRAY,
        ParameterDescriptor(
            description='Expected tool serial number per slot (same order as tools.index); '
            'empty string for an unassigned slot.'
        ),
    )
    node.declare_parameter(
        TOOLS_NODE_NAME,
        Parameter.Type.STRING_ARRAY,
        ParameterDescriptor(
            description='Endtool node name per slot (same order as tools.index); '
            'empty string for an unassigned slot.'
        ),
    )


def declare_simulation_parameters(node: Node) -> None:
    """Declare the initial simulated inventory parameters (``simulation.*``) on a node."""
    node.declare_parameter(
        SIMULATION_RACK_INDEX,
        Parameter.Type.INTEGER_ARRAY,
        ParameterDescriptor(description='Rack slot indices of the initial simulated rack inventory.'),
    )
    node.declare_parameter(
        SIMULATION_RACK_TAG_DATA,
        Parameter.Type.STRING_ARRAY,
        ParameterDescriptor(
            description='Hex-encoded RFID payload per simulated slot (same order as '
            'simulation.tool_rack.index); empty string for an empty slot.'
        ),
    )
    node.declare_parameter(
        SIMULATION_TOOL_MOUNT,
        '',
        ParameterDescriptor(
            description='Hex-encoded RFID payload of the initially mounted tool; empty string for no tool.'
        ),
    )


def _read_array(node: Node, name: str, required: bool) -> list:
    try:
        return list(node.get_parameter(name).value)
    except ParameterUninitializedException:
        if required:
            raise ValueError(f'required parameter {name!r} is not set') from None
        return []


def read_tools_config(node: Node) -> ToolsManagerConfigDTO:
    """Read and validate the expected tool assignments from the node parameters.

    :raises ValueError: If a parameter is missing or the assignments are inconsistent.
    """
    return ToolsManagerConfigDTO.from_parameters(
        index=_read_array(node, TOOLS_INDEX, required=True),
        tool_sn=_read_array(node, TOOLS_TOOL_SN, required=True),
        node_name=_read_array(node, TOOLS_NODE_NAME, required=True),
    )


def read_simulation_inventory(node: Node) -> SimulationInventoryDTO:
    """Read and validate the initial simulated inventory from the node parameters.

    Unset arrays mean an empty rack and an unset mount means no mounted tool.

    :raises ValueError: If the arrays are inconsistent or a payload is invalid.
    """
    return SimulationInventoryDTO.from_parameters(
        rack_index=_read_array(node, SIMULATION_RACK_INDEX, required=False),
        rack_tag_data=_read_array(node, SIMULATION_RACK_TAG_DATA, required=False),
        mount_tag_data=node.get_parameter(SIMULATION_TOOL_MOUNT).get_parameter_value().string_value,
    )
