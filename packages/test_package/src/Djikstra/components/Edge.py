class Edge:
    def __init__(self, node_a, node_b, weight):
        self.node_a = node_a
        self.node_b = node_b
        self.weight = weight

    def __str__(self):
        return f"Edge(node={self.node_a}, node={self.node_b}, weight={self.weight})"

    def __repr__(self):
        return self.__str__()