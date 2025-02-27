from enum import Enum

class TileType(Enum):
    STRAIGHT = "ST"
    CURVE = "CU"
    THREE_WAY_INTERSECTION = "3W"
    FOUR_WAY_INTERSECTION = "4W"