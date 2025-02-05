from packages.test_package.src.Djikstra.Map import DUCKIETOWN_CITY
from packages.test_package.src.Djikstra.components.Graph import Graph

graph = Graph()
graph.generate_from_map(DUCKIETOWN_CITY)

print(graph.nodes)
print(graph.edges)

print(f"Nodes: {len(graph.nodes)}, it should be 30.")
print(f"Edges: {len(graph.edges)}, it should be 33.")