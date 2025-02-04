class Node:
    def __init__(self, id, coordinate, value):
        self.id = id
        self.coordinate = coordinate
        self.value = value

    def __str__(self):
        return f"Node(id={self.id}, coordinate={self.coordinate}, value={self.value})"

    def __repr__(self):
        return self.__str__()