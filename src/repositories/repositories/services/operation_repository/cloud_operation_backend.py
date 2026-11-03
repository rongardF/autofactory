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
"""Cloud operation repository backend with an intentionally stubbed transport.

The service contract is a GET to
``{base}/{station_id}/operation_repository/operation_data`` with assembly,
operation and schema-version query parameters. A successful response contains
one operation JSON object; a missing operation is a service failure.
"""

import json
from urllib.parse import quote, urlencode

from repositories.enums.operation_repository import SCHEMA_VERSION
from repositories.interfaces.operation_repository.operation_backend import (
    OperationBackend,
)
from repositories.models.operation_repository.operation_dto import OperationDTO


class CloudOperationBackend(OperationBackend):
    """Cloud-backed source; HTTP transport is not yet available."""

    def __init__(self, station_id: str, operation_repository_url: str) -> None:
        self._station_id = station_id
        self._base_url = operation_repository_url.rstrip('/')

    def get_operation(self, operation_id: str, assembly_id: str) -> OperationDTO:
        """Request and parse an operation from the cloud service."""
        query = urlencode(
            {
                'assembly_id': assembly_id,
                'operation_id': operation_id,
                'schema_version': SCHEMA_VERSION,
            }
        )
        url = (
            f'{self._base_url}/{quote(self._station_id)}'
            f'/operation_repository/operation_data?{query}'
        )
        payload = self._http_get(url)
        return OperationDTO.from_operation_data(json.loads(payload))

    def _http_get(self, url: str) -> bytes:
        """Raise a clear error until the cloud transport is implemented."""
        raise NotImplementedError(f'cloud mode not implemented (GET {url})')
