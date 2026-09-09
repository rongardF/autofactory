from endtools.model import ToolMetadataInfoDTO

_TOOLS_METADATA = [
    ToolMetadataInfoDTO(
        tool_ref="volumetric_dispenser",
        launch_file="dispensing_tool_launch.py",
        model_path="/endtools/model/dispensing_tool/",
    )
]



def get_tool_info(tool_ref: str) -> ToolMetadataInfoDTO:
    """
    Get the metadata for a given tool reference."""
    for tool in _TOOLS_METADATA:
        if tool.tool_ref == tool_ref:
            return tool

    raise ValueError(f"Tool reference '{tool_ref}' not found in metadata.")
