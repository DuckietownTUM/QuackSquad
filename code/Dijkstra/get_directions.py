from Dijkstra import Dijkstra



class get_directions:
    def __init__(self, graph, end):
        self.dijkstra = Dijkstra(graph)
        self.shortest_path = self.dijkstra.get_shortest_path(end)


    def define_directions(node,tile_where_we_are,next_tile):
                cur_x, cur_y = tile_where_we_are.coordinate
                next_x, next_y = next_tile.coordinate
                node_x, node_y = node.coordinate

                if cur_x == next_x or cur_y == next_y:
                    return "STRAIGHT"
                # elif ((cur_y == next_y - 1) and ((cur_x <= next_x - 1) or (cur_x >= next_x + 1))) or ((cur_y == next_y + 1) and ((cur_x <= next_x - 1) or (cur_x >= next_x + 1))):
                elif cur_x == node_x:
                    if ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                        return "RIGHT"
                    elif ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                        return "LEFT"
                # elif ((cur_x == next_x - 1) and ((cur_y <= next_y - 1) or (cur_y >= next_y + 1))) or ((cur_x == next_x + 1) and ((cur_y <= next_y - 1) or (cur_y >= next_y + 1))):
                elif cur_y == node_y:
                    if ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                        return "RIGHT"
                    elif ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                        return "LEFT"