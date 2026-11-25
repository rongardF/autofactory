# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS WARRANTIES ARE DISCLAIMED.
"""Cached material record served by the ``material_repository`` node."""

import json
from typing import Any

from pydantic import BaseModel, Field

from repositories.enums.material_repository.material_type_enum import MaterialTypeEnum
from repositories.models.material_repository.fluid_material import FluidMaterial
from repositories.models.material_repository.material import COMMON_MATERIAL_FIELDS
from repositories.models.material_repository.screw_material import ScrewMaterial
from repositories.msg import MaterialRepositoryMaterialData

_MATERIAL_MODELS = {
    MaterialTypeEnum.FLUID: FluidMaterial,
    MaterialTypeEnum.SCREW: ScrewMaterial,
}


def material_key(product_number: str, product_revision: str) -> tuple[str, str]:
    """Return the case-insensitive lookup key of a material.

    Args:
        product_number: The material product number.
        product_revision: The material product revision.

    Returns:
        tuple[str, str]: The normalized (number, revision) pair.
    """
    return (product_number.strip().casefold(), product_revision.strip().casefold())


class MaterialDataDTO(BaseModel, frozen=True):
    """A validated material exactly as it is served by the repository."""

    product_number: str = Field(description='Product number of the material.')
    product_revision: str = Field(description='Product revision of the material.')
    material_type: MaterialTypeEnum = Field(description='Type of the material.')
    is_serialized: bool = Field(
        description='True for serialized material, false for bulk material.'
    )
    payload_json: str = Field(
        description='Type-specific payload as a compact JSON object string.'
    )

    @property
    def key(self) -> tuple[str, str]:
        """The case-insensitive (number, revision) lookup key."""
        return material_key(self.product_number, self.product_revision)

    def to_msg(self) -> MaterialRepositoryMaterialData:
        """Convert the record to a ``MaterialRepositoryMaterialData`` message."""
        msg = MaterialRepositoryMaterialData()
        msg.product_number = self.product_number
        msg.product_revision = self.product_revision
        msg.material_type = self.material_type.value
        msg.is_serialized = self.is_serialized
        msg.payload_json = self.payload_json
        return msg

    @classmethod
    def from_file_entry(cls, entry: Any) -> 'MaterialDataDTO':
        """Strictly validate one entry of the local materials file.

        The entry holds the common fields plus a nested ``payload_json`` object.
        ``fluid`` payloads must validate against :class:`FluidMaterial`; ``screw``
        payloads only have to be JSON objects.

        Args:
            entry: One element of the materials file list.

        Returns:
            MaterialDataDTO: The validated record.

        Raises:
            ValueError: If a common field is missing or invalid, the material
                type is unknown, or the payload is invalid.
        """
        if not isinstance(entry, dict):
            raise ValueError('material entry must be a JSON object')
        missing = sorted((COMMON_MATERIAL_FIELDS | {'payload_json'}) - entry.keys())
        if missing:
            raise ValueError(f'missing fields: {", ".join(missing)}')

        try:
            material_type = MaterialTypeEnum(entry['material_type'])
        except ValueError as error:
            raise ValueError(
                f"unknown material_type {entry['material_type']!r}"
            ) from error

        payload = entry['payload_json']
        if not isinstance(payload, dict):
            raise ValueError('payload_json must be a JSON object')

        common = {name: entry[name] for name in COMMON_MATERIAL_FIELDS}
        # Raises pydantic.ValidationError (a ValueError) for an invalid entry.
        _MATERIAL_MODELS[material_type].model_validate({**payload, **common})

        return cls(
            product_number=entry['product_number'],
            product_revision=entry['product_revision'],
            material_type=material_type,
            is_serialized=entry['is_serialized'],
            payload_json=json.dumps(payload, separators=(',', ':')),
        )
