from typing import List

from Tile import Tile
from TileType import TileType


class Map:
    def __init__(self, tiles: List[List[Tile]]):
        self.tiles = tiles

    def __str__(self):
        return "\n".join(
            " ".join(str(tile) if tile is not None else "  " for tile in row)
            for row in self.tiles
        )

DUCKIETOWN_CITY = Map(
    [
        [None, None, Tile(TileType.CURVE), Tile(TileType.STRAIGHT), Tile(TileType.STRAIGHT), Tile(TileType.CURVE)],
        [None, Tile(TileType.CURVE), Tile(TileType.CURVE), None, None, Tile(TileType.STRAIGHT)],
        [Tile(TileType.CURVE), Tile(TileType.CURVE), None, Tile(TileType.CURVE), Tile(TileType.STRAIGHT), Tile(TileType.THREE_WAY_INTERSECTION)],
        [Tile(TileType.STRAIGHT), None, None, Tile(TileType.STRAIGHT), None, Tile(TileType.STRAIGHT)],
        [Tile(TileType.THREE_WAY_INTERSECTION), Tile(TileType.STRAIGHT), Tile(TileType.STRAIGHT), Tile(TileType.FOUR_WAY_INTERSECTION), Tile(TileType.STRAIGHT), Tile(TileType.THREE_WAY_INTERSECTION)],
        [Tile(TileType.STRAIGHT), None, None, Tile(TileType.STRAIGHT), None, Tile(TileType.STRAIGHT)],
        [Tile(TileType.CURVE), Tile(TileType.STRAIGHT), Tile(TileType.STRAIGHT), Tile(TileType.THREE_WAY_INTERSECTION), Tile(TileType.STRAIGHT), Tile(TileType.CURVE)]
    ]
)