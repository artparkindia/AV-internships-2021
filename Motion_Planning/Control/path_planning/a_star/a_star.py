"""
Module for A* algorithm

Reference Code: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py
"""

from path_planning import *
from .heuristics import *
from .parameters import *
from .statistics import *
import numpy as np

class AStar(PathPlanning):
    """
    Module for the A* Path Planning algorithm
    """
    def __init__(self):
        # Initialize constants
        self.RESOLUTION = 10
        self.WIDTH = 1072
        self.HEIGHT = 1072

        self.parameters = [
            MotionModelParameter(self),
            HeuristicFunctionParameter(self),
            WeightParameter(self),
            CostFunctionParameter(self)
        ]

        self.statistics = [
            PathLength(),
            Iterations()
        ]

    def plan_path(self, obstacles, start, end):
        """ A star path planning """
        # Statistics
        computation_count = 0
        path_length = 0

        start_node = Node()
        start_node.x = self._calculate_cartesian_index(start[0])
        start_node.y = self._calculate_cartesian_index(start[1])
        
        goal_node = Node()
        goal_node.x = self._calculate_cartesian_index(end[0])
        goal_node.y = self._calculate_cartesian_index(end[1])

        open_set, closed_set = dict(), dict()
        open_set[self._calculate_grid_index(start_node)] = start_node

        while len(open_set) != 0:
            current_node_index = min(open_set, 
            key = lambda i: self.COST_FUNCTION(goal_node, open_set[i], open_set[i].cost, self.HEURISTIC, self.WEIGHT))

            current_node = open_set[current_node_index]

            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                goal_node.parent_index = current_node.parent_index
                goal_node.cost = current_node.cost
                break

            # Remove from open set and add to closed set
            del open_set[current_node_index]
            closed_set[current_node_index] = current_node

            for motion in self.MOTION_MODEL:
                node = Node()
                node.x = current_node.x + motion[0]
                node.y = current_node.y + motion[1]
                node.cost = current_node.cost + motion[2]
                node.parent_index = current_node_index
                
                node_index = self._calculate_grid_index(node)

                # Check validity of the node
                if not self._check_validity(node, obstacles):
                    continue

                if node_index in closed_set:
                    continue

                if node_index not in open_set:
                    open_set[node_index] = node
                else:
                    if open_set[node_index].cost > node.cost:
                        open_set[node_index] = node

            computation_count += 1

        points = self._calculate_final_path(goal_node, closed_set)

        path_length = len(points)
        self.statistics[0].set_value(computation_count)
        self.statistics[1].set_value(path_length)

        points.reverse()
        return points

    def _check_validity(self, node, obstacles):
        """ Private function to check the validity of the node """
        px = self._calculate_grid_position(node.x)
        py = self._calculate_grid_position(node.y)

        # Check constraint
        if px < 0:
            return False
        elif py < 0:
            return False
        elif px >= self.WIDTH:
            return False
        elif py >= self.HEIGHT:
            return False

        # Check collision
        collision_node = Node()
        collision_node.x = px; collision_node.y = py
        if not obstacles.check_collision(collision_node):
            return False

        return True


    def _calculate_final_path(self, goal_node, closed_set):
        """ Function to calculate the final path """
        points = [[self._calculate_grid_position(goal_node.x), self._calculate_grid_position(goal_node.y)]]
        parent_index = goal_node.parent_index

        while parent_index != -1:
            node = closed_set[parent_index]
            points.append([self._calculate_grid_position(node.x), self._calculate_grid_position(node.y)])
            parent_index = node.parent_index

        return points

    def _calculate_grid_position(self, index):
        """ Private function to calculate the position on grid from index """
        position = index * self.RESOLUTION
        return position     

    def _calculate_cartesian_index(self, position):
        """ Private Function to calculate the xy index from position """
        return round(position / self.RESOLUTION)

    def _calculate_grid_index(self, node):
        """ Private Function to calculate the grid index from a node """
        return node.y * self.WIDTH + node.x