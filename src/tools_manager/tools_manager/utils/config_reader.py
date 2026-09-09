from pathlib import Path
from yaml import safe_load, YAMLError

from pydantic import ValidationError

from tools_manager.model.rack_config import RackConfigDTO


def read_tool_rack_config_file(file_path: str) -> RackConfigDTO:
    """Read, parse, and validate the tool-rack config YAML file."""
    if not file_path:
        raise ValueError('config file path is empty')

    path = Path(file_path)

    try:
        data = safe_load(path.read_text())
    except OSError as exc:
        raise ValueError(f'could not read config file {path}: {exc}') from exc
    except YAMLError as exc:
        raise ValueError(f'malformed YAML in {path}: {exc}') from exc

    try:
        return RackConfigDTO.model_validate(data)
    except ValidationError as exc:
        raise ValueError(f'invalid config in {path}: {exc}') from exc