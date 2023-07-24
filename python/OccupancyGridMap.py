import numpy as np
from utils import Vertex, Vertices, heuristic


OCCUPIED = 1
FREE = 0

def get_manhattan_movements(x: int, y: int) -> list:
    """
    get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(x + 1, y + 0),
            (x + 0, y + 1),
            (x - 1, y + 0),
            (x + 0, y - 1)]

def get_euclidean_movements(x: int, y: int) -> list:
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

class OccupancyGridMap2D():
    def __init__(self, x_dim:int, y_dim:int, 
                 moves_type:str, known_obs_list:list=None) -> None:
        """
        OccupancyGridMap class is a 2D grid map with x_dim * y_dim cells.

        param inputs are as follows:
        - x_dim: x dimension of the map
        - y_dim: y dimension of the map
        - moves_type: type of moves, which can be either 'Manhattan' or 'Euclidean'
        - known_obs_list: list of tuples of known obstacles (obs_x, obs_y)

        The map is initialized with all free space (0), obstacles are set to 1. 
        """
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.map_bounds = (self.x_dim, self.y_dim)
        self.moves_type = moves_type
        self.known_obs_list = known_obs_list

        self.__init_map()
        self.visited = {}
        self.obstacle_set = set()

        # Set initial obstacles in the map if we have any
        if self.known_obs_list is not None:
            self.__set_init_obstacles(self.known_obs_list)

    def __init_map(self)->None:
        self.occupancy_grid_map = np.zeros(self.map_bounds, dtype=np.uint8)
    
    def __set_init_obstacles(self, obst_list:list) -> None:
        """
        Set initial obstacles in the map.
        """
        for obs in obst_list:
            self.set_obstacle_to_map(obs)

    def set_obstacle_to_map(self, pos:(int, int)) -> None:
        """
        Set an obstacle in the map, where 1 is obstacle and 0 is free space.
        """
        if pos not in self.obstacle_set:
            self.obstacle_set.add(pos)

        self.occupancy_grid_map[round(pos[0]), round(pos[1])] = OCCUPIED
        
    def remove_obstacle(self, pos: (int, int)) -> None:
        """
        removes an obstacle from the map at the given position of the obstacle
        """
        if pos in self.obstacle_set:
            self.obstacle_set.remove(pos)

        self.occupancy_grid_map[round(pos[0]), round(pos[1])] = FREE
        
    def get_map(self) -> np.ndarray:
        """
        Return the current configuration space of the map
        """
        return self.occupancy_grid_map
    
    def set_map(self, new_occ_map:np.ndarray) -> None:
        """
        Set the current configuration space of the map
        """
        self.occupancy_grid_map = new_occ_map

    def is_unoccupied(self, pos:(int,int))-> bool:
        """
        Check if a position is unoccupied.

        Performance improvement:
        Update this to have a set of obstacles to check against instead 
        of checking the map for faster performance.
        If the obstacle is removed from the set though we need to also do this for 
        the occupancy grid map.
        """
        if pos in self.obstacle_set:
            return True
        
        return False

        # if self.occupancy_grid_map[int(pos[0]), int(pos[1])] == FREE:
        #     return True
        # else:
        #     return False
    
    def in_bounds(self, cell: (int,int)) -> None:
        """
        Check if a cell is within the map bounds, return True if it is.
        """
        x,y = cell
        if (0 <= x < self.x_dim) and (0 <= y < self.y_dim):
            return True 
        
    def get_successors(self, vertex:(int,int)) -> list:
        """
        Get successors by making moves from the current vertex to the next.
        Does this by checking if the next vertex is within the map bounds and
        is unoccupied.
        """ 
        x,y = vertex
        if self.moves_type == 'Manhattan': 
            movements = get_manhattan_movements(x=x, y=y)
        else:
            movements = get_euclidean_movements(x=x, y=y)

        legal_movements = []
        for movement in movements:
            if self.in_bounds(movement) and self.is_unoccupied(movement):
                legal_movements.append(movement)
        
        return legal_movements

    def local_observation(self, global_position: (int, int), view_range: int = 2) -> dict:
        """
        :param global_position: position of robot in the global map frame
        :param view_range: how far ahead we should look
        :return: dictionary of new observations
        """
        (px, py) = global_position
        nodes_list = []
        for x in range(px - view_range, px + view_range + 1):
            for y in range(py - view_range, py + view_range + 1):
                if self.in_bounds((x, y)):
                    nodes_list.append((x, y))

        obs_dict = {}
        for node in nodes_list:
            if self.is_unoccupied(pos=node):
                obs_dict[node] = FREE
            else:
                obs_dict[node] = OCCUPIED
        return obs_dict

class SLAM2D():
    def __init__(self, occup_map: OccupancyGridMap2D, view_range: int):
        self.ground_truth_map = occup_map 
        self.slam_map = OccupancyGridMap2D(x_dim=occup_map.x_dim,
                                           y_dim=occup_map.y_dim, 
                                           moves_type=occup_map.moves_type,
                                           known_obs_list=occup_map.known_obs_list)
        self.view_range = view_range

    def set_ground_truth_map(self, gt_map: OccupancyGridMap2D) -> None:
        """update the ground truth map"""
        self.ground_trut    h_map = gt_map
        print("length of ground truth obstacle set: ", len(self.ground_truth_map.obstacle_set)) 
        print("obstacles are: ", len(self.ground_truth_map.obstacle_set )) 


    def compute_cost(self, parent:(int,int), child:(int,int)) -> float:
        """
        computes the cost between parent to child node
        returns: euclidean distance to traverse. inf if obstacle in path
        """
        if not self.slam_map.is_unoccupied(child) or \
        not self.slam_map.is_unoccupied(parent):
            return np.inf
        
        else:
            return heuristic(parent, child)

    def update_changed_edge_costs(self, local_grid: dict) -> Vertices:
        """
        This function updates the cost of the edges of the vertices that have
        changed in the local grid. It returns a list of vertices that have
        changed. 

        This is done by first checking if the local grid has any new obstacles
        or if any obstacles have been removed. If there are new obstacles, then
        we add the new obstacles to the slam map and add the vertices of the
        new obstacles to the list of changed vertices. If there are obstacles
        that have been removed, then we remove the obstacles from the slam map
        and add the vertices of the removed obstacles to the list of changed
        vertices.
        """
        vertices = Vertices()
        for node, value in local_grid.items():
            # if obstacle
            if value == OCCUPIED:
                # update slam map with obstacle if not already there
                if self.slam_map.is_unoccupied(node):
                    parent = Vertex(pos=node)
                    successors = self.slam_map.get_successors(node)
                    for child in successors:
                        parent.add_edge_with_cost(succ=child, 
                                                  cost=self.compute_cost(child, parent.pos))
                    vertices.add_vertex(parent)
                    self.slam_map.set_obstacle_to_map(node)
            else:
                # if not occupied then check if obstacle in slam map and remove 
                if not self.slam_map.is_unoccupied(node):
                    parent = Vertex(pos=node)
                    successors = self.slam_map.get_successors(node)
                    for child in successors:
                        parent.add_edge_with_cost(succ=child, cost=self.compute_cost(child, parent.pos))
                    vertices.add_vertex(parent)
                    self.slam_map.remove_obstacle(node)
        return vertices

    def rescan(self, global_position: (int, int)) -> (Vertices, OccupancyGridMap2D):
        """
        This method is called when the robot has moved to a new position. It 
        rescans the local area and updates the slam map with the new vertices
        and edges. It returns the list of changed vertices and the slam map.
        """
        # rescan local area
        local_observation = self.ground_truth_map.local_observation(
            global_position=global_position,view_range=self.view_range)

        vertices = self.update_changed_edge_costs(local_grid=local_observation)
        return vertices, self.slam_map


if __name__ == "__main__":
    x_dim = 50
    y_dim = 50
    static_obstacles = [(10, 10), (10, 11), (10, 12), (10, 13), (10, 14)]
    moves_type = 'Euclidean'
    
    world = OccupancyGridMap2D(x_dim=x_dim,
                               y_dim=y_dim,
                               moves_type=moves_type,
                               known_obs_list=static_obstacles)

    new_map = world 
    old_map = world
    # set up SLAM
    view_range = 2
    slam = SLAM2D(occup_map=world, view_range=view_range)
    slam.set_ground_truth_map(gt_map=world)

    start_position = (1, 0)
    new_position = start_position
    last_position = start_position

    slam_map_info = []
    for i in range(10):
        new_position = (new_position[0] + 1, new_position[1])

        if new_position!=last_position:
            last_position = new_position
            
            obstacle_position = (new_position[0] + 1, new_position[1]+1)

            #set a new obstacle, this is done by sensing the new obstacle and updating the slam map 
            world.set_obstacle_to_map(obstacle_position)
            new_world = world

            #need to update slam map with new obstacle
            slam.set_ground_truth_map(gt_map=new_world)

            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            slam_map_info.append(slam_map.occupancy_grid_map)

    print("SLAM MAP")


