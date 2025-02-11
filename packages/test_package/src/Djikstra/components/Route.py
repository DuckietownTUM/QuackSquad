class Route:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def __str__(self):
        return f"Route(from={self.start}, to={self.end})"

    def __repr__(self):
        return self.__str__()

