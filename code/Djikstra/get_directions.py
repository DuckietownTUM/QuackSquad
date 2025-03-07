from Dijkstra import Dijkstra



class get_directions:
    def __init__(self, graph, end):
        self.dijkstra = Dijkstra(graph)
        self.shortest_path = self.dijkstra.get_shortest_path(end)



    def define_directions(self):
        for i, node in enumerate(self.shortest_path):
            if node.value == "3W" or node.value == "4W":
                tile_where_we_are = self.shortest_path[i - 1]
                next_tile = self.shortest_path[i + 1]
                cur_x = tile_where_we_are.x
                cur_y = tile_where_we_are.y
                next_x = next_tile.x
                next_y = next_tile.y

                if cur_x == next_x or cur_y == next_y:
                    return "STRAIGHT"
                elif ((cur_y == next_y - 1) and ((cur_x <= next_x - 1) or (cur_x >= next_x + 1))) or ((cur_y == next_y + 1) and ((cur_x <= next_x - 1) or (cur_x >= next_x + 1))):
                    if ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                        return "RIGHT"
                    elif ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                        return "LEFT"
                elif ((cur_x == next_x - 1) and ((cur_y <= next_y - 1) or (cur_y >= next_y + 1))) or ((cur_x == next_x + 1) and ((cur_y <= next_y - 1) or (cur_y >= next_y + 1))):
                    if ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                        return "RIGHT"
                    elif ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                        return "LEFT"