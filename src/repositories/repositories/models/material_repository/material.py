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
"""Base model shared by every material type.

Concrete materials (``FluidMaterial``, ``ScrewMaterial``) are imported by the
nodes that consume material data, so they carry no ``DTO`` suffix.
"""

import json
from typing import Any, Self

from pydantic import BaseModel, ConfigDict, Field, StrictBool

from repositories.enums.material_repository.material_type_enum import MaterialTypeEnum
from repositories.msg import MaterialRepositoryMaterialData

# Fields every material carries; everything else is the type-specific payload.
COMMON_MATERIAL_FIELDS = frozenset(
    {'product_number', 'product_revision', 'material_type', 'is_serialized'}
)


class Material(BaseModel):
    """Common (mandatory) fields of every material plus its payload.

    The model is frozen and has no default values: every value must come from
    the material repository. Unknown payload fields are accepted and preserved
    so that newer repository data stays readable by older consumers.
    """

    model_config = ConfigDict(frozen=True, extra='allow')

    product_number: str = Field(
        min_length=1,
        description='Product (part) number of the material; with the revision it '
        'is the unique key of a material.',
    )
    product_revision: str = Field(
        min_length=1,
        description='Product (part) revision of the material.',
    )
    material_type: MaterialTypeEnum = Field(
        description='Material type; defines the schema of the payload fields.',
    )
    is_serialized: StrictBool = Field(
        description='True for serialized material (with a serial number), false '
        'for bulk material.',
    )

    def to_msg(self) -> MaterialRepositoryMaterialData:
        """Convert the material to a ``MaterialRepositoryMaterialData`` message.

        Returns:
            MaterialRepositoryMaterialData: The message; ``payload_json`` holds
            every non-common field (including preserved extra fields).
        """
        payload = self.model_dump(mode='json', exclude=set(COMMON_MATERIAL_FIELDS))
        msg = MaterialRepositoryMaterialData()
        msg.product_number = self.product_number
        msg.product_revision = self.product_revision
        msg.material_type = self.material_type.value
        msg.is_serialized = self.is_serialized
        msg.payload_json = json.dumps(payload, separators=(',', ':'))
        return msg

    @classmethod
    def from_msg(cls, msg: MaterialRepositoryMaterialData) -> Self:
        """Build the material from a ``MaterialRepositoryMaterialData`` message.

        Args:
            msg: The message to convert.

        Returns:
            Self: The validated material.

        Raises:
            ValueError: If ``payload_json`` is not a JSON object, or the data
                does not validate against this model
                (:class:`pydantic.ValidationError` is a ``ValueError``).
        """
        try:
            payload: Any = json.loads(msg.payload_json)
        except json.JSONDecodeError as error:
            raise ValueError(f'payload_json is not valid JSON: {error}') from error
        if not isinstance(payload, dict):
            raise ValueError('payload_json must be a JSON object')
        return cls.model_validate(
            {
                **payload,
                'product_number': msg.product_number,
                'product_revision': msg.product_revision,
                'material_type': msg.material_type,
                'is_serialized': msg.is_serialized,
            }
        )
