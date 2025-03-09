from components.Graph import Graph
from get_directions import get_directions
from Map import DUCKIETOWN_CITY
from components.Route import Route
from Dijkstra import Dijkstra
from TileType import TileType

graph = Graph()


graph.generate_from_map(DUCKIETOWN_CITY)

start_coordinate = (2, 0)
to_coordinate = (3,5)
start = next((node for node in graph.nodes if node.coordinate == start_coordinate))
to = next((node for node in graph.nodes if node.coordinate == to_coordinate))
route = Route(start, to)

dijkstra = Dijkstra(graph)
shortest_path = dijkstra.get_shortest_path(route)
for i, node in enumerate(shortest_path):
    if node.value.type.value=="3W" or node.value.type.value=="4W":
        tile_where_we_are = shortest_path[ i- 1]
        next_tile = shortest_path[i + 1]
        print(get_directions.define_directions(node,tile_where_we_are,next_tile))




print(start)
print(to)

print(f"To go from {start_coordinate} to {to_coordinate} we need {len(shortest_path)-1} tile!")
for node in shortest_path:
    print(node.coordinate)
