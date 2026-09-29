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
"""Unit tests for the config loader (SPEC section 6)."""

import textwrap
from pathlib import Path

import pytest

from io_controllers.enums import DeviceTypeEnum
from io_controllers.utils.config_loader import ConfigError, load_config

_VALID_YAML = textwrap.dedent(
    """
    device_type: ur
    publish_rate_hz: 25.0
    device:
      controller_namespace: /io_and_status_controller
      mapping:
        digital_in_states:
          - {pin: 0, io_name: part_present, default: false}
    """
)


def _write(tmp_path: Path, name: str, content: str) -> str:
    path = tmp_path / name
    path.write_text(content)
    return str(path)


def test_load_valid_config(tmp_path: Path):
    cfg = load_config(_write(tmp_path, 'ur.yaml', _VALID_YAML))
    assert cfg.device_type is DeviceTypeEnum.UR
    assert cfg.publish_rate_hz == pytest.approx(25.0)
    assert cfg.device['controller_namespace'] == '/io_and_status_controller'


def test_default_publish_rate_applied(tmp_path: Path):
    yaml_text = 'device_type: ur\ndevice: {}\n'
    cfg = load_config(_write(tmp_path, 'min.yaml', yaml_text))
    assert cfg.publish_rate_hz == pytest.approx(10.0)


def test_empty_path_rejected():
    with pytest.raises(ConfigError, match='empty'):
        load_config('')


def test_missing_absolute_file_rejected(tmp_path: Path):
    missing = str(tmp_path / 'nope.yaml')
    with pytest.raises(ConfigError, match='not found'):
        load_config(missing)


def test_malformed_yaml_rejected(tmp_path: Path):
    path = _write(tmp_path, 'bad.yaml', 'device_type: ur\n  bad: : :\n')
    with pytest.raises(ConfigError, match='malformed YAML'):
        load_config(path)


def test_non_mapping_document_rejected(tmp_path: Path):
    path = _write(tmp_path, 'list.yaml', '- a\n- b\n')
    with pytest.raises(ConfigError, match='top-level mapping'):
        load_config(path)


def test_unknown_device_type_rejected(tmp_path: Path):
    path = _write(tmp_path, 'x.yaml', 'device_type: fanuc\ndevice: {}\n')
    with pytest.raises(ConfigError, match='invalid config'):
        load_config(path)


def test_missing_device_block_rejected(tmp_path: Path):
    path = _write(tmp_path, 'nodev.yaml', 'device_type: ur\n')
    with pytest.raises(ConfigError, match='invalid config'):
        load_config(path)
