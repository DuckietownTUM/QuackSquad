from components.Graph import Graph
from components.Route import Route
from Map import DUCKIETOWN_CITY
from Dijkstra import Dijkstra


def get_direction(current_node, intersection_node, next_node):
    cur_x, cur_y = current_node.coordinates
    inter_x, inter_y = intersection_node.coordinates
    next_x, next_y = next_node.coordinates

    if cur_x == next_x or cur_y == next_y:
        return "STRAIGHT"
    elif cur_x == inter_x:
        if ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
            return "RIGHT"
        else:
            return "LEFT"
    elif cur_y == inter_y:
        if ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
            return "RIGHT"
        else:
            return "LEFT"

graph = Graph()
graph.generate_from_map(DUCKIETOWN_CITY)

start_coordinates = (2, 0)
to_coordinates = (3, 5)
start = next((node for node in graph.nodes if node.coordinates == start_coordinates))
to = next((node for node in graph.nodes if node.coordinates == to_coordinates))
route = Route(start, to)

dijkstra = Dijkstra(graph)
shortest_path = dijkstra.get_shortest_path(route)

print("\nNavigator by Dijkstra")
print(f"Start: {start}")
print(f"To: {to}")

print(f"\nTo go from {start_coordinates} to {to_coordinates} we need {len(shortest_path) - 1} tiles:")
for i, node in enumerate(shortest_path):
    print(node.coordinates, end="")
    if node.value.type.value == "3W" or node.value.type.value == "4W":
        current_node = shortest_path[i - 1]
        intersection_node = shortest_path[i]
        next_node = shortest_path[i + 1]
        direction = get_direction(current_node, intersection_node, next_node)
        print(f" (go {direction})")
    else:
        print()