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
"""Local material source backed by a JSON materials file."""

import json
from pathlib import Path
from typing import Any

from repositories.exceptions.material_repository.material_not_found_error import (
    MaterialNotFoundError,
)
from repositories.interfaces.material_repository.material_backend import (
    MaterialBackend,
)
from repositories.models.material_repository.material_data_dto import (
    MaterialDataDTO,
    material_key,
)


class LocalMaterialBackend(MaterialBackend):
    """Parse, strictly validate and cache every material of a JSON file."""

    def __init__(self, materials_file_path: str) -> None:
        self._materials_file_path = Path(materials_file_path)
        self._materials: dict[tuple[str, str], MaterialDataDTO] = {}

    def load(self) -> int:
        """Read the materials file into the in-memory mapping.

        Any invalid entry fails the whole load: nothing is cached in that case.

        Returns:
            int: The number of materials loaded.

        Raises:
            ValueError: If the file is missing, unreadable, not valid JSON, not a
                list, or contains an invalid or duplicate material.
        """
        try:
            with self._materials_file_path.open(encoding='utf-8') as file:
                content: Any = json.load(file)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise ValueError(
                f'Unable to read materials file '
                f'{str(self._materials_file_path)!r}: {error}'
            ) from error
        if not isinstance(content, list):
            raise ValueError('Materials file root must be a list of materials.')

        materials: dict[tuple[str, str], MaterialDataDTO] = {}
        for index, entry in enumerate(content):
            try:
                material = MaterialDataDTO.from_file_entry(entry)
                if material.key in materials:
                    raise ValueError(
                        f'duplicate material {material.product_number!r} '
                        f'revision {material.product_revision!r}'
                    )
            except ValueError as error:
                raise ValueError(
                    f'Invalid material at index {index}: {error}'
                ) from error
            materials[material.key] = material

        self._materials = materials
        return len(materials)

    def get_material(
        self, product_number: str, product_revision: str
    ) -> MaterialDataDTO:
        """Look up one cached material (case-insensitive exact match)."""
        try:
            return self._materials[material_key(product_number, product_revision)]
        except KeyError as error:
            raise MaterialNotFoundError('material not found') from error
