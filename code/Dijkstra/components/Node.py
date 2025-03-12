class Node:
    def __init__(self, id, coordinates, value):
        self.id = id
        self.coordinates = coordinates
        self.value = value

    def __eq__(self, other):
        return isinstance(other, Node) and self.id == other.id

    def __hash__(self):
        return hash(self.value)

    def __str__(self):
        return f"Node(id={self.id}, coordinates={self.coordinates}, value={self.value})"

    def __repr__(self):
        return self.__str__()