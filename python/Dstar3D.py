from priority_queue import PriorityQueue, Priority
import numpy as np
from utils import heuristic_3d, Vertex, Vertices, get_movements_16n
from typing import Dict, List
from multipledispatch import dispatch
import matplotlib.pyplot as plt
import random 
"""
Vertex and Nodes are the same 
Edges and lines are the same 

Probably better to inflate radius and check in?
Computation slow for check of collision

Remember that I am going backwards so from goal 
position back to start position

"""

UNOCCUPIED = 0
N_OBSTACLES = 35
OBSTACLE_ON = True
OBSTACLE_RADIUS = 2 #radius of obstacle
ROBOT_RADIUS = 1 # radius of robot

class GridMap3D():
    def __init__(self, x_dim:int=125, y_dim:int=125, z_dim:int=5) -> None:
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim

        self.obstacle_set = set()
        
        #set random obstacles 
        random.seed(0)
        if OBSTACLE_ON == True:
            for i in range(N_OBSTACLES):
                x = random.randint(15,x_dim-25)
                y = random.randint(15,y_dim-25)
                z = random.randint(0,z_dim)
                self.obstacle_set.add((x,y,z))
                self.inflate_self((x,y,z), 
                                  OBSTACLE_RADIUS, 
                                  self.obstacle_set)

    def inflate_self(self, position:(int,int,int), 
                     radius:int, set_to_add:set) -> None:
        """
        inflates the obstacle by the radius 
        """
        (x,y,z) = position
        for i in range(-radius-1,radius+1):
            for j in range(-radius-1,radius+1):
                for k in range(-radius-1,radius+1):
                    set_to_add.add((x+i,y+j,z+k))

    def get_successors(self, node_pos: (int, int,int), 
                       avoid_obstacles: bool = True) -> list:
        """
        returns all successors or predecessors based on 
        move type you can make, this method checks if 
        """
        (x,y,z) = node_pos
        #right now this does diagonal movements 8 directions
        movements = get_movements_16n(x,y,z)
        # not needed. Just makes aesthetics to the path
        # if (x + y + z) % 2 == 0: movements.reverse()

        legal_moves = self.get_legal_moves(
            movements, avoid_obstacles)
        
        return legal_moves

    def get_legal_moves(
            self, move_list:list, avoid_obstacles:bool) -> list:
        """
        returns legal movements based on the movement list
        and if we want to avoid obstacles or not
        """
        legal_moves = []

        if avoid_obstacles == True:
            for move in move_list:
                if self.is_collision(move) or self.is_out_bounds(move):
                    continue
                legal_moves.append(move)
        else:
            for move in move_list:
                if self.is_out_bounds(move):
                    continue
                
                legal_moves.append(move)

        return legal_moves 

    def is_collision(self, current_move:(int,int)) -> bool:
        """
        checks if the current_move will be within the obstacle set or 
        if within the robots radius, returns true if so 
        returns false if not 

        https://en.wikipedia.org/wiki/Collision_detection#Optimization
        """
        
        #consider radius of robot 
        inflated_moves = set()
        self.inflate_self(current_move, ROBOT_RADIUS, inflated_moves)

        for move in inflated_moves:
            if move in self.obstacle_set:
                return True
        
        return False
    
    def  is_out_bounds(self,move:(int,int,int)) -> bool:
        """
        checks if the move is outside the bounds of the grid
        returns true if so, false if not 
        """
        
        (x,y,z) = move
        
        if x <= 0 or x >= self.x_dim:
            return True
        if y <= 0 or y >= self.y_dim:
            return True        
        if z <= 0 or z >= self.z_dim:
            return True

        return False


class Dstarlite3D:
    def __init__(self, grid_map: GridMap3D, 
                 s_start: (int, int, int), 
                 s_goal: (int, int, int)):
        """
        :param map: the ground truth map of the environment provided by gui
        :param s_start: start location
        :param s_goal: end location
        """
        self.new_edges_and_old_costs = None

        # algorithm start
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0  # accumulation
        self.U = PriorityQueue()

        """
        The rhs-values are one-step lookahead values based 
        on the g-values and thus potentially better informed 
        than the g-values
        """
        self.rhs = np.ones((grid_map.x_dim, 
                            grid_map.y_dim,
                            grid_map.z_dim)) * np.inf #initialize as inf

        """
        Estimate of the cost to get from the start to the goal 
        through a given vertex
        """
        self.g = self.rhs.copy() #set g == rhs to make map consistent

        #this is cl
        self.sensed_map = GridMap3D(x_dim=grid_map.x_dim,
                                  y_dim=grid_map.y_dim, 
                                  z_dim=grid_map.z_dim)
        
        self.rhs[self.s_goal] = 0 #set goal point rhs to 0
        self.U.insert(self.s_goal, Priority(heuristic_3d(self.s_start, self.s_goal), 0))

    def calculate_key(self, s: (int, int, int)) -> Priority:
        """
        :param s: the vertex we want to calculate key
        :return: Priority class of the two keys
        From page 3 in the D* '
        For k1 = Get the minimum between g and rhs 
        then add the heuristic and the accumulated cost
        For k2 = Get the minimum between g and rhs
        Return the two keys as a Priority class object
        """
        k1 = min(self.g[s], self.rhs[s]) + heuristic_3d(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)

    def compute_cost(self, u: (int, int,int), v: (int, int, int)) -> float:
        """
        calcuclate the cost between nodes
        :param u: from vertex
        :param v: to vertex
        :return: euclidean distance to traverse. inf if obstacle in path
        """
        if (self.sensed_map.is_out_bounds(u) == True) or \
            (self.sensed_map.is_collision(v) == True): 
            return float('inf')
        else:
            return heuristic_3d(u, v)

    def contain(self, u: (int, int,int)) -> list:
        """check if u in Priority queue heap"""
        return u in self.U.vertices_in_heap

    def update_vertex(self, u: (int, int, int)) -> None:
        # and priority queue has the node u, update priority value
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.U.update(u, self.calculate_key(u))
        #if doesn't exist then insert 
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.U.insert(u, self.calculate_key(u))
        # already in t
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.U.remove(u)

    def compute_shortest_path(self) -> None:
        """
        U.Top() returns the VERTEX/NODE with the smallest priority of ALL 
        vertices/nodes in the Priority Queue U. 
        
        U.Top_key() returns the SMALLEST PRIORITY of ALL vertices/nodes 
        in the Priority Queue U.
        """
        
        while (self.U.top_key() < self.calculate_key(self.s_start)) or \
            (self.rhs[self.s_start] > self.g[self.s_start]):

            u = self.U.top()
            k_old = self.U.top_key()
            #calculate 
            k_new = self.calculate_key(u)
            # so first compare the priority value
            # compare using the __le__ operator 
            # we compare to see if k1 < other k1 or  
            # if k_old_k1 == k_old_k2 then check k2
            if k_old < k_new:
                self.U.update(u, k_new)

            # otherwise we compare the cost of the node 
            # location between g and rhs (look ahead) 
            # if g is higher than rhs we need to update 
            # rhs since costs don't match up 
            # will need to remove from queue and recalculate 
            # the previous nodes that connected to u 
            # and reupdate the values for the rhs 
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                # print("u: ", u)
                # print("g elif: ", self.g[u])
                self.U.remove(u)
                pred = self.sensed_map.get_successors(node_pos=u)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s] = min(self.rhs[s], self.compute_cost(s, u) + self.g[u])
                    self.update_vertex(s)
               
            # we have new territory to explore 
            # basically get all predecessors
            # for each node in predecessor
            # check to see rhs[pred] equals to the cost 
            # from s_prime to current node + old cost 
            # from current node if so, check all 
            else:
                self.g_old = self.g[u]
                self.g[u] = float('inf')
                # print("g else: ", self.g[u])
                pred = self.sensed_map.get_successors(node_pos=u)
                pred.append(u)
                for s in pred:
                    if self.rhs[s] == self.compute_cost(s, u) + self.g_old:
                        if s != self.s_goal:
                            #update costs for rhs duplication -> return min_s
                            min_s = float('inf')
                            succ = self.sensed_map.get_successors(node_pos=s)
                            for s_ in succ:
                                temp = self.compute_cost(s, s_) + self.g[s_]
                                if min_s > temp:
                                    min_s = temp
                            # update rhs with min_s found
                            self.rhs[s] = min_s
                    self.update_vertex(u)

                # print("g", self.g[u])

    def rescan(self) -> Vertices:

        new_edges_and_old_costs = self.new_edges_and_old_costs
        self.new_edges_and_old_costs = None
        return new_edges_and_old_costs

    def main(self, start_position: (int, int, int)) -> None:
        path = [start_position]
        self.s_start = start_position
        self.s_last = self.s_start
        self.compute_shortest_path()

        while self.s_start != self.s_goal:
            assert (
                self.rhs[self.s_start] != float('inf')), "There is no known path!"
            #get all valid successors
            succ = self.sensed_map.get_successors(self.s_start)
            min_s = float('inf')
            arg_min = None

            #loop through each successor and compute cost duplication
            # print("\n")
            # print("outside loop")
            for s_ in succ:
                # print("s_: ", s_)
                temp = self.compute_cost(self.s_start, s_) + self.g[s_]

                #update cost only if less than mins
                if temp < min_s:
                    min_s = temp
                    arg_min = s_

            ### algorithm sometimes gets stuck here for some reason !!! FIX
            self.s_start = arg_min
            print("start: ", self.s_start)
            path.append(self.s_start)

        return path, self.g, self.rhs


if __name__ == '__main__':
    start = (1,1,2)
    goal = (90,90,3)
    map_grid  = GridMap3D()
    dstar = Dstarlite3D(grid_map=map_grid, 
                      s_start=start, 
                      s_goal=goal)
    
    path,g_costs,rhs_cost = dstar.main(start_position=start)

    x_path = [x[0] for x in path]
    y_path = [y[1] for y in path]
    z_path = [z[2] for z in path]

    #plot path
    fig,ax = plt.subplots()
    ax.plot(x_path,y_path, '-o', color='red', label='path')
    ax.plot(goal[0],goal[1], 'o', color='green', label='goal')
    ax.plot(start[0],start[1], 'o', color='blue', label='start')

    #plot obstacles
    x_obs = [x[0] for x in map_grid.obstacle_set]
    y_obs = [y[1] for y in map_grid.obstacle_set]
    ax.plot(x_obs,y_obs, 'o', color='black', label='obstacles')

    ax.legend()
    plt.show()

    #plot 3d
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.plot3D(x_path,y_path,z_path, '-o', color='red', label='path')
    ax2.plot3D(goal[0],goal[1],goal[2], 'o', color='green', label='goal')
    ax2.plot3D(start[0],start[1],start[2], 'o', color='blue', label='start')

    #plot obstacles
    x_obs = [x[0] for x in map_grid.obstacle_set]
    y_obs = [y[1] for y in map_grid.obstacle_set]
    z_obs = [z[2] for z in map_grid.obstacle_set]
    ax2.plot3D(x_obs,y_obs,z_obs, 'o', 
               color='c', label='obstacles', alpha=0.1)
    plt.show()

