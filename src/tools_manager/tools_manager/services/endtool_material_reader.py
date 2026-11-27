"""Reading the material of the configured endtools from their read-only parameters."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.lifecycle import LifecycleNode
from station_utils.services.node_manager import NodeManager

from tools_manager.model.endtool_material_dto import EndtoolMaterialDTO
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolSlotDTO

MATERIAL_PRODUCT_NUMBER = 'material.product_number'
MATERIAL_PRODUCT_REVISION = 'material.product_revision'

# A freshly created client can lose its first response to a DDS discovery race,
# so a failed read is attempted once more before giving up.
_READ_ATTEMPTS = 2


def _get_parameter(manager: NodeManager, name: str) -> ParameterValue | None:
    """Read one parameter of the managed node, retrying once on failure."""
    for _ in range(_READ_ATTEMPTS):
        result = manager.get_parameter(name)
        if result.success:
            return result.value
    return None


def read_endtool_materials(
    node: LifecycleNode,
    slots: Sequence[ToolSlotDTO],
    timeout_sec: float = 5.0,
) -> dict[str, EndtoolMaterialDTO]:
    """Read the material parameters of the endtool of every assigned slot.

    The endtool nodes only have to be running; their material parameters are
    read-only and available while the node is unconfigured.

    :param node: Owning node used for the parameter service clients.
    :param slots: Expected tool assignments; unassigned slots are skipped.
    :param timeout_sec: Timeout for service availability and results per call.
    :returns: The material of every configured tool, keyed by tool serial number.
    :raises ValueError: If an endtool cannot be read or has no material set.
    """
    materials: dict[str, EndtoolMaterialDTO] = {}
    for slot in slots:
        if not slot.tool_sn or not slot.node_name:
            continue  # intentionally left empty slot
        manager = NodeManager(node, slot.node_name, service_timeout_s=timeout_sec)
        try:
            product_number = _get_parameter(manager, MATERIAL_PRODUCT_NUMBER)
            product_revision = _get_parameter(manager, MATERIAL_PRODUCT_REVISION)
        finally:
            manager.destroy()

        for name, value in (
            (MATERIAL_PRODUCT_NUMBER, product_number),
            (MATERIAL_PRODUCT_REVISION, product_revision),
        ):
            if value is None:
                raise ValueError(
                    f"cannot read parameter '{name}' of endtool '{slot.node_name}' (tool '{slot.tool_sn}'); "
                    'make sure the endtool node is running'
                )
            if value.type != ParameterType.PARAMETER_STRING or not value.string_value:
                raise ValueError(
                    f"parameter '{name}' of endtool '{slot.node_name}' (tool '{slot.tool_sn}') is not set"
                )
        assert product_number is not None and product_revision is not None
        materials[slot.tool_sn] = EndtoolMaterialDTO(
            product_number=product_number.string_value,
            product_revision=product_revision.string_value,
        )
    return materials


def with_material(tool: ToolInfoDto, materials: Mapping[str, EndtoolMaterialDTO]) -> ToolInfoDto:
    """Return the tool with the material of its endtool.

    A tool whose serial number is not configured gets an empty material.
    """
    material = materials.get(tool.tool_sn)
    return tool.model_copy(
        update={
            'material_part_number': material.product_number if material else '',
            'material_part_revision': material.product_revision if material else '',
        }
    )
