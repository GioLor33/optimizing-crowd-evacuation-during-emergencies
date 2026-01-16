class Node():
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.edges = {}  # dictionary with [key, value] as [neighbor_node.id, cost]