from pydantic import BaseModel
from typing import Optional

class BaseConfig(BaseModel):
    """
    Define Base Configuration for all Configuration
    """
    name: Optional[str]
    pass
