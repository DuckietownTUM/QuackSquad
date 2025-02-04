from packages.test_package.src.Djikstra.components.Node import Node
from packages.test_package.src.Djikstra.components.Edge import Edge

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

        for x in range(0, len(map.tiles)):
            for y in range(0, len(map.tiles[x])):
                tile = map.tiles[x][y]
                if tile:
                    id = y * len(map.tiles[x]) + x
                    node = Node(id, (x, y), tile)
                    self.nodes.append(node) # FIX: the node could already exists
                    for k in range(0, 4):
                        direction = directions[k]
                        neighbor_x = x + direction[0]
                        neighbor_y = y + direction[1]
                        if 0 <= neighbor_x < len(map.tiles) and 0 <= neighbor_y < len(map.tiles[x]):
                            neighbor_tile = map.tiles[neighbor_x][neighbor_y]
                            if neighbor_tile:
                                neighbor_id = neighbor_y * len(map.tiles[neighbor_x]) + neighbor_x
                                if x < neighbor_x or y < neighbor_y:
                                    neighbor_node = Node(neighbor_id, (neighbor_x, neighbor_y), neighbor_tile)
                                    self.nodes.append(neighbor_node)
                                else:
                                    neighbor_node = next((node for node in self.nodes if node.id == neighbor_id))
                                edge = next((edge for edge in self.edges if edge.node_a == neighbor_node and edge.node_b == node), None)
                                if not edge:
                                    edge = Edge(node, neighbor_node, 1)
                                    self.edges.append(edge)