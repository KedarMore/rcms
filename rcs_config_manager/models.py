from pydantic import BaseModel
from typing import List
from datetime import datetime

class ConfigType(BaseModel):
    static: dict
    dynamic: dict
    adaptive: dict

class FileData(BaseModel):
    nodes: list[str]
    config_name: str
    config: ConfigType = {}
    
class PostData(BaseModel):
    getAll:bool = True
    status:bool = False
    data: FileData = None
    date:str = (datetime.now()).strftime("%d-%m-%Y")
    timestamp:str = (datetime.now()).strftime("%H:%M:%S")

class Config(BaseModel):
    config:list = []
    # config_version:str = ""
    active_layout_id:int = -1
    timestamp:int = 0


# class FileData(BaseModel):
#     config: list[dict]
#     timestamp: str
#     active_layout_id: int

# class Config(BaseModel):
#     config_name: str
#     config_type: dict
#     nodes: list[str]

# class config_type(BaseModel):
#     dynamic: dict
#     static: dict
#     adaptive: dict

