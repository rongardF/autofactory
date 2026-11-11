# Copyright (c) 2026, Autofactory
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Loader of the localization poses YAML file."""

from pathlib import Path

import yaml
from pydantic import ValidationError

from localization.models.localization_poses_dto import LocalizationPosesDTO


def load_poses_file(path: str) -> LocalizationPosesDTO:
    """Load and validate the poses file.

    The file holds ``poses: [[x, y, z, roll, pitch, yaw], ...]``.

    :raises ValueError: If the file is missing, is not valid YAML or holds no valid poses.
    """
    file = Path(path)
    if not file.is_file():
        raise ValueError(f"poses file '{path}' does not exist")
    try:
        content = yaml.safe_load(file.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"poses file '{path}' could not be read: {exc}") from exc
    if not isinstance(content, dict) or 'poses' not in content:
        raise ValueError(f"poses file '{path}' must contain a 'poses' list")
    try:
        return LocalizationPosesDTO(poses=content['poses'])
    except ValidationError as exc:
        raise ValueError(f"poses file '{path}' is invalid: {exc}") from exc
