from collections import deque, namedtuple
import math


# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')
Node = namedtuple('Node', 'node, x, y')

def make_edge(start, end, cost=1):
  return Edge(start, end, cost)

def make_node(node, x, y):
    return Node(node, x, y)

class Astar:
    def __init__(self, edges, nodes):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]
        self.nodes = [make_node(*node) for node in nodes]

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    def heuristic(self, start, end):
        return math.sqrt((start.x - end.x)**2 + (start.y - end.y)**2)

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            node_start = self.find_node(edge.start)
            node_end = self.find_node(edge.end)
            neighbours[edge.start].add((edge.end, edge.cost + self.heuristic(node_start, node_end)))
        return neighbours
    def find_node(self, value):
        length = len(self.nodes)
        i = 0
        while (i<length and self.nodes[i].node!=value):
            i = i + 1
        return self.nodes[i]
        # print(len(self.nodes))

    def astar(self, source, dest):
        cost = 0
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return distances[dest]

class Dijkstra:
    def __init__(self, edges):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        cost = 0
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return distances[dest]

if __name__ == "__main__":

    graph_dijkstra = Dijkstra([
    ("S", "A1", 44.78),  ("S", "A2", 36),  ("S", "A3", 29.11), ("S", "A4", 21.3),
    ("S", "A5", 14.16), ("S", "A6", 6.25), ("S", "A7", 6.25),  ("S", "A8", 15.33),
    ("A1", "S", 44.78),  ("A2", "S", 36),  ("A3", "S", 29.11), ("A4", "S", 21.3),
    ("A5", "S", 14.16), ("A6", "S", 6.25), ("A7", "S", 6.25),  ("A8", "S", 15.33),
    ("A1", "B1", 7.5), ("B1", "A1", 7.5), ("A2", "B1", 9), ("B1", "A2", 9), 
    ("A2", "B2", 5), ("B2", "A2", 5), ("A3", "B2", 9), ("B2", "A3", 9),
    ("A3", "B3", 5), ("B3", "A3", 5), ("A4", "B3", 9), ("B3", "A4", 9),
    ("A4", "B4", 5), ("B4", "A4", 5), ("A5", "B4", 9), ("B4", "A5", 9),
    ("A5", "B5", 5), ("B5", "A5", 5), ("A6", "B5", 9), ("B5", "A6", 9),
    ("A6", "B6", 5), ("B6", "A6", 5), ("A7", "B7", 5), ("B7", "A7", 5),
    ("A7", "B8", 12.3), ("B8", "A7", 12.3), ("A8", "B8", 7.5), ("B8", "A8", 7.5),
    ("B1", "C1", 7.5), ("C1", "B1", 7.5), ("B1", "B2", 5), ("B2", "B1", 5),
    ("B2", "C1", 9), ("C1", "B2", 9), ("B2", "C2", 7.5), ("C2", "B2", 7.5),
    ("B2", "B3", 7.5), ("B3", "B2", 7.5), ("B3", "C2", 8.3), ("C2", "B3", 8.3), 
    ("B3", "C3", 5), ("C3", "B3", 5), ("B3", "B4", 7.5), ("B4", "B3", 7.5),
    ("B4", "C3", 9), ("C3", "B4", 9), ("B4", "C4", 5), ("C4", "B4", 5),
    ("B4", "B5", 7.5), ("B5", "B4", 7.5), ("B5", "C4", 9), ("C4", "B5", 9),
    ("B5", "B6", 7.5), ("B6", "B5", 7.5), ("B6", "D3", 20), ("D3", "B6", 20),
    ("B7", "D4", 20), ("D4", "B7", 20), ("B7", "C5", 17), ("C5", "B7", 17),
    ("B8", "C5", 5), ("C5", "B8", 5), ("C1", "C2", 5), ("C2", "C1", 5),
    ("C2", "C3", 5), ("C3", "C2", 5), ("C3", "C4", 5), ("C4", "C3", 5),
    ("C4", "D1", 10), ("D1", "C4", 10), ("C4", "D2", 12.5), ("D2", "C4", 12.5),
    ("C5", "D5", 17), ("D5", "C5", 17), ("C5", "D5", 17), ("D5", "C5", 17),
    ("D1", "D2", 7.5), ("D2", "D1", 7.5), ("D2", "D3", 7.5), ("D3", "D2", 7.5),
    ("D3", "E1", 5), ("E1", "D3", 5), ("D3", "D4", 10.5), ("D4", "D3", 10.5),
    ("D3", "E2", 11.6), ("E2", "D3", 11.6), ("D4", "E1", 11.6), ("E1", "D4", 11.6),
    ("D4", "E2", 5), ("E2", "D4", 5), ("D4", "E3", 9), ("E3", "D4", 9),
    ("D4", "D5", 5), ("D5", "D4", 5), ("D5", "E3", 7.5), ("E3", "D5", 7.5),
    ("E1", "F1", 5), ("F1", "E1", 5), ("E1", "F2", 9), ("F2", "E1", 9),
    ("E2", "F1", 9), ("F1", "E2", 9), ("E2", "F2", 5), ("F2", "E2", 5),
    ("E2", "E3", 7.5), ("E3", "E2", 7.5), ("E3", "F2", 9), ("F2", "E3", 9),
    ("F1", "G1", 5), ("G1", "F1", 5), ("F2", "G1", 11.6), ("G1", "F2", 11.6), ])  

    node_available = ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", 
        "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8",
        "C1", "C2", "C3", "C4", "C5", 
        "D1", "D2", "D3", "D4", "D5",
        "E1", "E2", "E3",
        "F1", "F2",
        "G1"]  

    list_park_dijkstra = []
    while (len(node_available)>0):
        results = []
        for node in node_available:
            cost = graph_dijkstra.dijkstra("S", node)
            results.append([node, cost])
        cost_min = inf
        node_min = "S"
        for result in results:
            if (result[1]<cost_min):
                cost_min = result[1]
                node_min = result[0]
        list_park_dijkstra.append([node_min, cost_min])
        node_available.remove(node_min)
    print(list_park_dijkstra)

    # Untuk astar

    graph_astar = Astar([
    ("S", "A1", 44.78),  ("S", "A2", 36),  ("S", "A3", 29.11), ("S", "A4", 21.3),
    ("S", "A5", 14.16), ("S", "A6", 6.25), ("S", "A7", 6.25),  ("S", "A8", 15.33),
    ("A1", "S", 44.78),  ("A2", "S", 36),  ("A3", "S", 29.11), ("A4", "S", 21.3),
    ("A5", "S", 14.16), ("A6", "S", 6.25), ("A7", "S", 6.25),  ("A8", "S", 15.33),
    ("A1", "B1", 7.5), ("B1", "A1", 7.5), ("A2", "B1", 9), ("B1", "A2", 9), 
    ("A2", "B2", 5), ("B2", "A2", 5), ("A3", "B2", 9), ("B2", "A3", 9),
    ("A3", "B3", 5), ("B3", "A3", 5), ("A4", "B3", 9), ("B3", "A4", 9),
    ("A4", "B4", 5), ("B4", "A4", 5), ("A5", "B4", 9), ("B4", "A5", 9),
    ("A5", "B5", 5), ("B5", "A5", 5), ("A6", "B5", 9), ("B5", "A6", 9),
    ("A6", "B6", 5), ("B6", "A6", 5), ("A7", "B7", 5), ("B7", "A7", 5),
    ("A7", "B8", 12.3), ("B8", "A7", 12.3), ("A8", "B8", 7.5), ("B8", "A8", 7.5),
    ("B1", "C1", 7.5), ("C1", "B1", 7.5), ("B1", "B2", 5), ("B2", "B1", 5),
    ("B2", "C1", 9), ("C1", "B2", 9), ("B2", "C2", 7.5), ("C2", "B2", 7.5),
    ("B2", "B3", 7.5), ("B3", "B2", 7.5), ("B3", "C2", 8.3), ("C2", "B3", 8.3), 
    ("B3", "C3", 5), ("C3", "B3", 5), ("B3", "B4", 7.5), ("B4", "B3", 7.5),
    ("B4", "C3", 9), ("C3", "B4", 9), ("B4", "C4", 5), ("C4", "B4", 5),
    ("B4", "B5", 7.5), ("B5", "B4", 7.5), ("B5", "C4", 9), ("C4", "B5", 9),
    ("B5", "B6", 7.5), ("B6", "B5", 7.5), ("B6", "D3", 20), ("D3", "B6", 20),
    ("B7", "D4", 20), ("D4", "B7", 20), ("B7", "C5", 17), ("C5", "B7", 17),
    ("B8", "C5", 5), ("C5", "B8", 5), ("C1", "C2", 5), ("C2", "C1", 5),
    ("C2", "C3", 5), ("C3", "C2", 5), ("C3", "C4", 5), ("C4", "C3", 5),
    ("C4", "D1", 10), ("D1", "C4", 10), ("C4", "D2", 12.5), ("D2", "C4", 12.5),
    ("C5", "D5", 17), ("D5", "C5", 17), ("C5", "D5", 17), ("D5", "C5", 17),
    ("D1", "D2", 7.5), ("D2", "D1", 7.5), ("D2", "D3", 7.5), ("D3", "D2", 7.5),
    ("D3", "E1", 5), ("E1", "D3", 5), ("D3", "D4", 10.5), ("D4", "D3", 10.5),
    ("D3", "E2", 11.6), ("E2", "D3", 11.6), ("D4", "E1", 11.6), ("E1", "D4", 11.6),
    ("D4", "E2", 5), ("E2", "D4", 5), ("D4", "E3", 9), ("E3", "D4", 9),
    ("D4", "D5", 5), ("D5", "D4", 5), ("D5", "E3", 7.5), ("E3", "D5", 7.5),
    ("E1", "F1", 5), ("F1", "E1", 5), ("E1", "F2", 9), ("F2", "E1", 9),
    ("E2", "F1", 9), ("F1", "E2", 9), ("E2", "F2", 5), ("F2", "E2", 5),
    ("E2", "E3", 7.5), ("E3", "E2", 7.5), ("E3", "F2", 9), ("F2", "E3", 9),
    ("F1", "G1", 5), ("G1", "F1", 5), ("F2", "G1", 11.6), ("G1", "F2", 11.6),],
    [("S", 40, 46), ("A1", 0, 40), ("A2", 5, 40), ("A3", 12.5, 40), ("A4", 20, 40),
    ("A5", 27.5, 40), ("A6", 35, 40), ("A7", 45.5, 40), ("A8", 50.5, 40),
    ("B1", 0, 32.5), ("B2", 5, 35), ("B3", 12.5, 35), ("B4", 20, 35), ("B5", 27.5, 35),
    ("B6", 35, 35), ("B7", 45.5, 35), ("B8", 50.5, 32.5), ("C1", 0, 25), ("C2", 5, 27.5),
    ("C3", 10, 30), ("C4", 15, 30), ("C5", 30.5, 25), ("D1", 15, 15), ("D2", 22.5, 15),
    ("D3", 30, 15), ("D4", 40.5, 15), ("D5", 45.5, 15), ("E1", 30, 10), ("E2", 40.5, 10),
    ("E3", 48, 10), ("F1", 30, 5), ("F2", 40.5, 5), ("G1", 35, 0) ])

    node_available = ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", 
        "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8",
        "C1", "C2", "C3", "C4", "C5", 
        "D1", "D2", "D3", "D4", "D5",
        "E1", "E2", "E3",
        "F1", "F2",
        "G1"]

    list_park_astar = []
    while (len(node_available)>0):
        results = []
        for node in node_available:
            cost = graph_astar.astar("S", node)
            results.append([node, cost])
        cost_min = inf
        node_min = "S"
        for result in results:
            if (result[1]<cost_min):
                cost_min = result[1]
                node_min = result[0]
        list_park_astar.append([node_min, cost_min])
        node_available.remove(node_min)

    print(list_park_astar)