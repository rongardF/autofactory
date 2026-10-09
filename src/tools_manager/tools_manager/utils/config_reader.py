from pathlib import Path
from yaml import safe_load, YAMLError

from pydantic import ValidationError

from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO


def read_config_file(file_path: str) -> ToolsManagerConfigDTO:
    """Read, parse, and validate the tools manager config YAML file."""
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
        return ToolsManagerConfigDTO.model_validate(data)
    except ValidationError as exc:
        raise ValueError(f'invalid config in {path}: {exc}') from exc