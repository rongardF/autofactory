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
"""Read-only data source interface of the ``material_repository`` node."""

from abc import ABC, abstractmethod

from repositories.models.material_repository.material_data_dto import MaterialDataDTO


class MaterialBackend(ABC):
    """A read-only source of material data."""

    @abstractmethod
    def load(self) -> int:
        """Prepare the source (for example parse and cache a file).

        Returns:
            int: The number of materials made available.

        Raises:
            ValueError: If the source is missing, unreadable or invalid.
        """
        raise NotImplementedError

    @abstractmethod
    def get_material(
        self, product_number: str, product_revision: str
    ) -> MaterialDataDTO:
        """Return the material with the given product number and revision.

        Raises:
            MaterialNotFoundError: If no material matches.
        """
        raise NotImplementedError
