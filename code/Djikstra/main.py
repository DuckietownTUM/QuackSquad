from packages.test_package.src.Djikstra.Map import DUCKIETOWN_CITY
from packages.test_package.src.Djikstra.components.Graph import Graph
from packages.test_package.src.Djikstra.components.Route import Route
from packages.test_package.src.Djikstra.Dijkstra import Dijkstra

graph = Graph()
graph.generate_from_map(DUCKIETOWN_CITY)

start_coordinate = (2, 0)
to_coordinate = (2,4)
start = next((node for node in graph.nodes if node.coordinate == start_coordinate))
to = next((node for node in graph.nodes if node.coordinate == to_coordinate))
route = Route(start, to)

dijkstra = Dijkstra(graph)
shortest_path = dijkstra.get_shortest_path(route)


print(start)
print(to)

print(f"To go from {start_coordinate} to {to_coordinate} we need {len(shortest_path)-1} tile!")
for node in shortest_path:
    print(node.coordinate)