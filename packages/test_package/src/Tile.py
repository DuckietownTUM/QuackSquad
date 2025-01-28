from packages.test_package.src.TileType import TileType

class Tile:
    def __init__(self, type: TileType):
        self.type = type

    def __str__(self):
        return self.type.value