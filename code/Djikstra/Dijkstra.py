from packages.test_package.src.Djikstra.components.Graph import Graph

class Dijkstra:

    def __init__(self, graph: Graph):
        self.graph = graph
        self.predecessors = {}
        self.distances = {}
        self.settled_nodes = []
        self.unsettled_nodes = []


    x_values, y_values = graph.get_all_tile_coordinates()
    def initialize_distance(self, starting_node):
        for node in self.graph.nodes:
            self.distances[node] = float('inf')

        self.distances[starting_node] = 0
        self.unsettled_nodes.append(starting_node)

    def get_adjacent_nodes(self, node):
        adjacent = []
        for edge in self.graph.edges:
            if edge.node_a == node:
                adjacent.append(edge.node_b)
            if edge.node_b == node:
                adjacent.append(edge.node_a)
        return adjacent

    def adjacent_nodes_evaluation(self, node):
        for edge in self.graph.edges:
            node1 = None

            if edge.node_a == node or edge.node_b == node:
                node1 = edge.node_b

            if node1 is not None and node1 not in self.settled_nodes:
                new_distance = self.distances[node] + edge.weight
                if new_distance < self.distances[node1]:
                    self.distances[node1] = new_distance
                    self.predecessors[node1] = node
                    self.unsettled_nodes.append(node1)

        self.unsettled_nodes.remove(node)
        self.settled_nodes.append(node)

    def execute(self, starting_node):
        self.initialize_distance(starting_node)

        while self.unsettled_nodes:
            current_node = self.get_lowest_distance_node()
            self.get_adjacent_nodes(current_node)
            self.adjacent_nodes_evaluation(current_node)

    def get_lowest_distance_node(self):
        lowest_distance_node = None
        lowest_distance = float('inf')

        for node in self.unsettled_nodes:
            node_distance = self.distances[node]
            if node_distance < lowest_distance:
                lowest_distance = node_distance
                lowest_distance_node = node

        return lowest_distance_node

    def get_path(self, target_node):
        path = []
        step = target_node

        if step not in self.predecessors:
            return path

        path.append(step)
        while step in self.predecessors:
            step = self.predecessors[step]
            path.append(step)

        path.reverse()
        return path

    def get_path_distance(self, target_node):
        return self.distances[target_node]

    def get_shortest_path(self, route):
        self.execute(route.start)

        for node in self.graph.nodes:
            self.get_adjacent_nodes(node)

        return self.get_path(route.end)


    def get_directions(self, tile_where_we_are, next_tile):
        cur_x = tile_where_we_are.x
        cur_y = tile_where_we_are.y
        next_x= next_tile.x
        next_y= next_tile.y

        if cur_x == next_x or cur_y== next_y:
            return "STRAIGHT"
        elif  ((cur_y==next_y-1) and ((cur_x<=next_x-1) or(cur_x>=next_x+1))) or ((cur_y==next_y+1) and ((cur_x<=next_x-1) or(cur_x>=next_x+1))):
            if ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)) :
                return "RIGHT"
            elif ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)) :
                return "LEFT"
        elif ((cur_x==next_x-1) and ((cur_y<=next_y-1) or(cur_y>=next_y+1))) or ((cur_x==next_x+1) and ((cur_y<=next_y-1) or(cur_y>=next_y+1))):
            if ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                return "RIGHT"
            elif ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                return "LEFT"
