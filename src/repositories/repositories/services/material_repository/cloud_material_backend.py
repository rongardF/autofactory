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
"""Cloud material backend; a stub that makes no network calls."""

from repositories.interfaces.material_repository.material_backend import (
    MaterialBackend,
)
from repositories.models.material_repository.material_data_dto import MaterialDataDTO


class CloudMaterialBackend(MaterialBackend):
    """Cloud-backed source; the REST service does not exist yet.

    Lifecycle transitions succeed, but every lookup fails with
    ``NotImplementedError`` so the node answers ``cloud mode not implemented``.
    """

    def __init__(
        self,
        station_id: str,
        material_service_url: str,
        request_timeout_sec: float,
    ) -> None:
        self._station_id = station_id
        self._base_url = material_service_url.rstrip('/')
        self._request_timeout_sec = request_timeout_sec

    def load(self) -> int:
        """Nothing to prepare in the stub."""
        return 0

    def get_material(
        self, product_number: str, product_revision: str
    ) -> MaterialDataDTO:
        """Raise until the cloud transport is implemented."""
        raise NotImplementedError('cloud mode not implemented')
