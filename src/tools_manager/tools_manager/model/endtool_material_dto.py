"""Material of an endtool, as read from the endtool's read-only material parameters."""

from pydantic import BaseModel, ConfigDict, Field


class EndtoolMaterialDTO(BaseModel):
    """Product number and revision of the material an endtool dispenses."""

    model_config = ConfigDict(frozen=True)

    product_number: str = Field(min_length=1, description='Product number of the endtool material.')
    product_revision: str = Field(min_length=1, description='Product revision of the endtool material.')
