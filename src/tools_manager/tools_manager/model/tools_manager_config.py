from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, model_validator

from tools_manager.utils.hex_to_string import hex_to_string
from tools_manager.model.tool_metadata_dto import ToolMetadataDto


class ToolSlotDTO(BaseModel):
    """A single tool slot defined on the rack."""

    index: int = Field(description='Rack slot index.')
    tool_sn: str|None = Field(description='Serial number of the tool expected to be installed to the slot.', default=None)
    metadata: ToolMetadataDto|None = Field(description='Metadata for the tool, used to configure the tool node.', default=None)
    parameters: dict[str, str|float|bool|list]|None = Field(
        description='Parameters for the tool, used to configure the tool node.',
        default=None
    )

    def __eq__(self, value: object) -> bool:
        return (
            isinstance(value, ToolSlotDTO)
            and self.index == value.index
            and self.tool_sn == value.tool_sn
        )


    @model_validator(mode='before')
    @classmethod
    def _valid_tools_parameters_value_types(cls, data: Any) -> Any:
        """Restrict ``parameters`` values to str, float, int, bool or list.

        When a value is a list, every element of that list must itself be an
        int, float, str or bool. Runs before Pydantic coercion so the raw
        input types provided by the caller are validated as-is.
        """
        if not isinstance(data, dict):
            return data
        tools_parameters = data.get('parameters')
        if tools_parameters is None:
            return data

        if not isinstance(tools_parameters, dict):
            raise ValueError(
                f'tools_parameters has invalid type {type(tools_parameters).__name__}; must be dict',
            )

        allowed_scalar_types = (str, float, int, bool)
        for key, value in tools_parameters.items():
            if isinstance(value, list):
                for item_index, item in enumerate(value):
                    if not isinstance(item, allowed_scalar_types):
                        raise ValueError(
                            f'parameters[{key!r}][{item_index}] has '
                            f'invalid type {type(item).__name__}; list items must be '
                            'int, float, str or bool',
                        )
            elif not isinstance(value, allowed_scalar_types):
                raise ValueError(
                    f'parameters[{key!r}] has invalid type '
                    f'{type(value).__name__}; must be str, float, int, bool or list',
                )
        return data

    # NOTE: this validator has to be below the validator above (_valid_tools_parameters_value_types)
    # to be run first, do not move it!
    @model_validator(mode='before')
    @classmethod
    def _clear_fields_when_no_tool(cls, data: Any) -> Any:
        """Force ``parameters`` and ``metadata`` to ``None`` when ``tool_sn`` is unset.

        A slot without an expected tool serial number must not carry tool
        configuration, so any provided ``parameters`` or ``metadata`` are
        cleared before model initialization.
        """
        if not isinstance(data, dict):
            return data
        if data.get('tool_sn') is None:
            data['parameters'] = None
            data['metadata'] = None
        return data


class ToolRackSimulationSetupDTO(BaseModel):
    """Tools setup for simulation bootup."""

    index: int = Field(description='Rack slot index.')
    tag_data: bytes|None = Field(description='Tools RFID tag data.', default=None)


class SimulationSetupDTO(BaseModel):
    """Tools setup for simulation bootup."""

    tool_rack: list[ToolRackSimulationSetupDTO] = Field(description='Tools setup for the tool rack.')
    tool_mount: bytes|None = Field(description='Tools RFID tag data.', default=None)


class ToolsManagerConfigDTO(BaseModel):
    """Tools manager configuration: installed tools and simulation bootup config."""

    slots: list[ToolSlotDTO] = Field(description='Tool slots defined on the rack.')
    simulation_setup: SimulationSetupDTO = Field(description='Tools spawned into simulation at bootup.')

    def _serial_from_tag_data(self, tag_data: bytes | None) -> str | None:
        """Extract the tool serial number encoded in an RFID tag payload.

        The tag payload is stored as a hex string; the serial number occupies
        bytes ``16:31`` of the decoded payload. Returns ``None`` when no tag data
        is present or the serial field is blank.
        """
        if not tag_data:
            return None
        decoded = hex_to_string(tag_data.decode())
        serial = decoded[ slice(0, 15)].replace('\x00', '').strip()
        return serial or None

    @model_validator(mode='after')
    def _no_duplicate_serial_numbers(self) -> 'ToolsManagerConfigDTO':
        """Reject duplicate tool serial numbers within slots and within the simulation tool rack."""
        slot_serials = [slot.tool_sn for slot in self.slots if slot.tool_sn is not None]
        sim_serials = [
            serial
            for entry in self.simulation_setup.tool_rack
            if (serial := self._serial_from_tag_data(entry.tag_data)) is not None
        ]

        for section, serials in (('slots', slot_serials), ('simulation_setup', sim_serials)):
            seen: set[str] = set()
            duplicates = {sn for sn in serials if sn in seen or seen.add(sn)}
            if duplicates:
                raise ValueError(f'duplicate tool serial number in {section}: {sorted(duplicates)}')
        return self

    @model_validator(mode='after')
    def _no_duplicate_indices(self) -> 'ToolsManagerConfigDTO':
        """Reject duplicate slot indices within slots and within the simulation tool rack."""
        for section, entries in (('slots', self.slots), ('simulation_setup', self.simulation_setup.tool_rack)):
            seen: set[int] = set()
            duplicates = {e.index for e in entries if e.index in seen or seen.add(e.index)}
            if duplicates:
                raise ValueError(f'duplicate index in {section}: {sorted(duplicates)}')
        return self

    @model_validator(mode='after')
    def _sim_setup_matches_slots(self) -> 'ToolsManagerConfigDTO':
        """Every slot must have exactly one matching simulation tool rack entry (by slot index)."""
        slot_indices = {slot.index for slot in self.slots}
        sim_indices = {entry.index for entry in self.simulation_setup.tool_rack}

        missing = slot_indices - sim_indices
        if missing:
            raise ValueError(f'missing simulation_setup entries for slots: {sorted(missing)}')

        extra = sim_indices - slot_indices
        if extra:
            raise ValueError(f'simulation_setup entries with no matching slot: {sorted(extra)}')

        return self

    def get_tool_parameters(self, tool_sn: str) -> dict[str, str|float|bool|int|list]:
        """Get the parameters for a given tool serial number.

        :param tool_sn: Serial number of the tool.
        :returns: The parameters for the tool, or ``None`` if no parameters are defined.
        """
        for entry in self.slots:
            if entry.tool_sn == tool_sn:
                if entry.parameters is not None:
                    return entry.parameters
                else:
                    return {}

        return {}

    def get_tool_metadata(self, tool_sn: str) -> ToolMetadataDto|None:
        """Get the metadata for a given tool serial number.

        :param tool_sn: Serial number of the tool.
        :returns: The metadata for the tool, or ``None`` if no metadata is defined.
        """
        for entry in self.slots:
            if entry.tool_sn == tool_sn:
                return entry.metadata

        return None

    def get_simulated_tag_data(self, tool_sn: str) -> bytes|None:
        """Get the simulated RFID tag data for a given tool serial number.

        :param tool_sn: Serial number of the tool.
        :returns: The simulated tag data for the tool, or ``None`` if no tag data is defined.
        """
        for entry in self.simulation_setup.tool_rack:
            if (serial := self._serial_from_tag_data(entry.tag_data)) == tool_sn:
                return entry.tag_data

        return None

