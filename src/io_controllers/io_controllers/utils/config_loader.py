# Copyright (c) 2026, Movement Controller Contributors
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
"""Load and validate the device-agnostic GPIO controller config file.

Reads a YAML file, resolves its path (absolute, else relative to the package's
installed ``share/io_controllers/config`` directory), and validates only the
device-agnostic envelope into an :class:`IOControllerConfigDTO`. The opaque
``device`` block and ``io_name`` uniqueness are the selected driver's concern.
"""

from pathlib import Path

import yaml
from pydantic import ValidationError

from ament_index_python.packages import get_package_share_directory

from io_controllers.models.io_config_dto import IOControllerConfigDTO


class ConfigError(Exception):
    """Raised when the config file cannot be read or fails validation."""


def _resolve_config_path(config_file: str) -> Path:
    """Resolve *config_file* to an existing path.

    Absolute paths are used as-is; relative paths are resolved against the
    installed ``share/io_controllers/config`` directory when available, falling
    back to the current working directory.
    """
    if not config_file:
        raise ConfigError('config_file parameter is empty; a path is required')

    candidate = Path(config_file)
    if candidate.is_absolute():
        if not candidate.is_file():
            raise ConfigError(f'config file not found: {candidate}')
        return candidate

    search_dirs: list[Path] = []
    try:
        
        share = Path(get_package_share_directory('io_controllers'))
        search_dirs.append(share / 'config')
    except Exception:
        # Package share not resolvable (e.g. running from source in tests).
        pass
    search_dirs.append(Path.cwd())

    for directory in search_dirs:
        resolved = directory / candidate
        if resolved.is_file():
            return resolved

    searched = ', '.join(str(d) for d in search_dirs)
    raise ConfigError(
        f'config file {config_file!r} not found (searched: {searched})'
    )


def load_config(config_file: str) -> IOControllerConfigDTO:
    """Load, parse, and validate the config file into the envelope DTO.

    :param config_file: Absolute path, or a name/relative path resolved against
        the package share config directory.
    :raises ConfigError: on a missing file, malformed YAML, a non-mapping
        document, or envelope validation failure (unknown ``device_type``,
        missing fields, bad ``publish_rate_hz``).
    """
    path = _resolve_config_path(config_file)

    try:
        raw_text = path.read_text()
    except OSError as exc:
        raise ConfigError(f'could not read config file {path}: {exc}') from exc

    try:
        data = yaml.safe_load(raw_text)
    except yaml.YAMLError as exc:
        raise ConfigError(f'malformed YAML in {path}: {exc}') from exc

    if not isinstance(data, dict):
        raise ConfigError(
            f'config file {path} must contain a top-level mapping, '
            f'got {type(data).__name__}'
        )

    try:
        return IOControllerConfigDTO(**data)
    except ValidationError as exc:
        raise ConfigError(f'invalid config in {path}: {exc}') from exc
