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
"""Internal material service of the ``material_repository`` node."""

from repositories.exceptions.material_repository.material_repository_error import (
    MaterialRepositoryError,
)
from repositories.interfaces.material_repository.material_backend import (
    MaterialBackend,
)
from repositories.models.material_repository.material_data_dto import MaterialDataDTO


class MaterialService:
    """Serves material lookups from a backend while it is initialized.

    The service is created on ``configure`` (not initialized), initialized on
    ``activate`` and de-initialized on ``deactivate``.
    """

    def __init__(self, backend: MaterialBackend) -> None:
        self._backend = backend
        self._initialized = False

    @property
    def is_initialized(self) -> bool:
        """Whether lookups are currently served."""
        return self._initialized

    def initialize(self) -> None:
        """Start serving lookups."""
        self._initialized = True

    def deinitialize(self) -> None:
        """Stop serving lookups."""
        self._initialized = False

    def get_material(
        self, product_number: str, product_revision: str
    ) -> MaterialDataDTO:
        """Return one material.

        Raises:
            MaterialRepositoryError: If the service is not initialized.
            MaterialNotFoundError: If no material matches.
            NotImplementedError: In cloud (stub) mode.
        """
        if not self._initialized:
            raise MaterialRepositoryError('material service is not initialized')
        return self._backend.get_material(product_number, product_revision)
