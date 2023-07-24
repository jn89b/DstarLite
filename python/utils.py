import math
from multipledispatch import dispatch

class Vertex:
    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.edges_and_costs = {}

    def add_edge_with_cost(self, succ: (int, int), cost: float) -> None:
        if succ != self.pos:
            self.edges_and_costs[succ] = cost

    @property
    def edges_and_c_old(self) -> dict:
        return self.edges_and_costs


class Vertices:
    def __init__(self):
        self.list = []

    def add_vertex(self, v: Vertex) -> None:
        self.list.append(v)

    @property
    def vertices(self) -> list:
        return self.list


def heuristic(p: (int, int), q: (int, int)) -> float:
    """
    Helper function to compute distance between two points.
    :param p: (x,y)
    :param q: (x,y)
    :return: euclidean distance
    """
    return compute_distance(p, q)

def heuristic_3d(p: (int, int,int), q: (int, int,int)) -> float:
    """
    Helper function to compute distance between two points.
    :param p: (x,y)
    :param q: (x,y)
    :return: euclidean distance
    """
    return compute_3d_distance(p, q)


def compute_distance(p: (int, int), q: (int, int)) -> float:
    """
    Computes the distance between two points. 
    :param p: (x,y)s
    :param q: (x,y)
    :return: euclidean distance
    """
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

def compute_3d_distance(p: (int, int,int), q: (int, int,int)) -> float:
    """
    Computes three dimensional distance between two points.
    :param p: (x,y,z)
    :param q: (x,y,z)
    :return: euclidean distance
    """
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2]-q[2])**2)

def get_movements_8n(x: int, y: int) -> list:
    """
    get all possible 8-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(x + 1, y + 0),
            (x + 0, y + 1),
            (x - 1, y + 0),
            (x + 0, y - 1),
            (x + 1, y + 1),
            (x - 1, y + 1),
            (x - 1, y - 1),
            (x + 1, y - 1)]


def get_movements_16n(x: int, y: int, z:int) -> list:
    """
    get all possible 16-connectivity movements.
    :return: list of movements with cost [(dx, dy, dz, movement_cost)]
    """
    movements = []
    max_displacement = 1
    x_start, y_start, z_start = x,y,z

    for dx in range(-max_displacement, max_displacement+1):
        for dy in range(-max_displacement, max_displacement+1):
            for dz in range(-max_displacement, max_displacement+1):
                x_new = x_start + dx
                y_new = y_start + dy
                z_new = z_start + dz
                if (x_new,y_new,z_new) != (x,y,z):
                    movements.append((x_new, y_new, z_new))

    return movements