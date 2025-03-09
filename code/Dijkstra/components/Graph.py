from .Node import Node
from .Edge import Edge


class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def generate_from_map(self, map):
        directions = {
            0: (-1, 0),  # Left
            1: (0, 1),  # Up
            2: (1, 0),  # Right
            3: (0, -1),  # Down
        }

        for y in range(0, len(map.tiles)):
            for x in range(0, len(map.tiles[y])):
                tile = map.tiles[y][x]
                if tile:
                    id = y * len(map.tiles[y]) + x
                    node = next((node for node in self.nodes if node.id == id), None)
                    if not node:
                        node = Node(id, (x, y), tile)
                        self.nodes.append(node)
                    for k in range(0, 4):
                        direction = directions[k]
                        neighbor_x = x + direction[1]
                        neighbor_y = y + direction[0]
                        if 0 <= neighbor_y < len(map.tiles) and 0 <= neighbor_x < len(map.tiles[y]):
                            neighbor_tile = map.tiles[neighbor_y][neighbor_x]
                            if neighbor_tile:
                                neighbor_id = neighbor_y * len(map.tiles[neighbor_y]) + neighbor_x
                                neighbor_node = next((node for node in self.nodes if node.id == neighbor_id), None)
                                if not neighbor_node:
                                    neighbor_node = Node(neighbor_id, (neighbor_x, neighbor_y), neighbor_tile)
                                    self.nodes.append(neighbor_node)
                                edge = Edge(node, neighbor_node, 1)
                                self.edges.append(edge)

    # def get_all_tile_coordinates(self):
    #     x_coords = []
    #     y_coords = []
    #
    #     for node in self.nodes:
    #         x, y = node.coordinate
    #         x_coords.append(x)
    #         y_coords.append(y)
    #
    #     return x_coords, y_coords

    def __str__(self):
        return f"Graph(nodes={self.nodes}, edges={self.edges})"

    def __repr__(self):
        return self.__str__()