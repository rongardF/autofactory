from __future__ import annotations

from pydantic import BaseModel, Field, model_validator

from endtools.enumerators.tool_type_enum import ToolTypeEnum

from tools_manager.model import Pose6D



class ToolSlotDTO(BaseModel):
    """A single tool slot defined on the rack."""

    index: int = Field(description='Rack slot index.')
    tool_sn: str = Field(description='Serial number of the tool assigned to the slot.')
    launch_file: str = Field(description='Launch file name that launches the tool node(s).')


class SimBootupDTO(BaseModel):
    """A tool spawned into simulation at bootup."""

    index: int = Field(description='Rack slot index.')
    tool_sn: str = Field(description='Serial number of the tool to spawn.')
    tool_type: ToolTypeEnum = Field(description='Type of the tool to spawn.')
    tool_part_number: str = Field(description='Part number of the tool to spawn.')
    tool_part_revision: str = Field(description='Part revision of the tool to spawn.')
    material_part_number: str = Field(description='Part number of the material used by the tool.')
    material_part_revision: str = Field(description='Part revision of the material used by the tool.')
    xacro_file: str = Field(description='Xacro description file for the tool.')


class RackConfigDTO(BaseModel):
    """Tool rack configuration: slot definitions and simulation bootup tools."""

    slots: list[ToolSlotDTO] = Field(description='Tool slots defined on the rack.')
    sim_bootup: list[SimBootupDTO] = Field(description='Tools spawned into simulation at bootup.')

    @model_validator(mode='after')
    def _no_duplicate_serial_numbers(self) -> 'RackConfigDTO':
        """Reject duplicate tool serial numbers within slots and within sim_bootup."""
        for section, entries in (('slots', self.slots), ('sim_bootup', self.sim_bootup)):
            seen: set[str] = set()
            duplicates = {e.tool_sn for e in entries if e.tool_sn in seen or seen.add(e.tool_sn)}
            if duplicates:
                raise ValueError(f'duplicate tool_sn in {section}: {sorted(duplicates)}')
        return self

    @model_validator(mode='after')
    def _no_duplicate_indices(self) -> 'RackConfigDTO':
        """Reject duplicate slot indices within slots and within sim_bootup."""
        for section, entries in (('slots', self.slots), ('sim_bootup', self.sim_bootup)):
            seen: set[int] = set()
            duplicates = {e.index for e in entries if e.index in seen or seen.add(e.index)}
            if duplicates:
                raise ValueError(f'duplicate index in {section}: {sorted(duplicates)}')
        return self

    @model_validator(mode='after')
    def _sim_bootup_matches_slots(self) -> 'RackConfigDTO':
        """Every slot must have exactly one matching sim_bootup entry (by slot index)."""
        slot_indices = {slot.index for slot in self.slots}
        bootup_indices = {bootup.index for bootup in self.sim_bootup}

        missing = slot_indices - bootup_indices
        if missing:
            raise ValueError(f'missing sim_bootup entries for slots: {sorted(missing)}')

        extra = bootup_indices - slot_indices
        if extra:
            raise ValueError(f'sim_bootup entries with no matching slot: {sorted(extra)}')

        return self

    def get_matched_tools(self) -> zip[tuple[ToolSlotDTO|None, SimBootupDTO|None]]:
        """Return slots and sim_bootup entries aligned pairwise by slot index.

        If a corresponding entry is missing in either list, None is returned for that entry.
        """
        indices = [slot.index for slot in self.slots]
        indices += [bootup.index for bootup in self.sim_bootup if bootup.index not in indices]
        bootup_by_index = {bootup.index: bootup for bootup in self.sim_bootup}
        slots_by_index = {slot.index: slot for slot in self.slots}
        matched_slots: list[ToolSlotDTO|None] = []
        matched_bootups: list[SimBootupDTO|None] = []
        for index in indices:
            matched_slots.append(slots_by_index.get(index, None))
            matched_bootups.append(bootup_by_index.get(index, None))

        return zip(matched_slots, matched_bootups)