"""
Module for A* algorithm

Reference Code: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py
"""

from path_planning import *
from .heuristics import *
from .parameters import *
from .statistics import *
import numpy as np
import networkx

class AStarX(PathPlanning):
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
            HeuristicXFunctionParameter(self)
        ]

        self.statistics = [
            PathLength()
        ]

    def plan_path(self, obstacles, start, end):
        """ A star path planning """
        # Statistics
        path_length = 0

        # Generate weights for graph
        grid_graph = networkx.grid_2d_graph(self.HEIGHT, self.WIDTH)
        grid_graph = self._initialize_weights(grid_graph)

        # Generate the grid without obstacles
        for x in range(self.HEIGHT):
            for y in range(self.WIDTH):
                if not self._check_validity(x, y, obstacles):
                    grid_graph.remove_node((x, y))

        astar_path = networkx.astar_path(grid_graph, tuple(start), tuple(end), 
                     heuristic = self.HEURISTICX, weight="cost")

        astar_path = astar_path[::10]
        path_length = len(astar_path)
        self.statistics[0].set_value(path_length)

        return astar_path

    def _check_validity(self, px, py, obstacles):
        """ Private function to check the validity of the node """
        # Check collision
        collision_node = Node()
        collision_node.x = px; collision_node.y = py
        if not obstacles.check_collision(collision_node):
            return False

        return True

    def _initialize_weights(self, grid_graph):
        """ Function to generate weights for grid based graph """
        networkx.set_edge_attributes(grid_graph, {e: 1 for e in grid_graph.edges()}, "cost")

        if self.MOTION_MODEL == MOTION_4WAY:
            return grid_graph
        
        weight = 0
        if self.MOTION_MODEL == MOTION_8WAY:
            weight = np.sqrt(2)
        elif self.MOTION_MODEL == MOTION_8WAYLESS:
            weight = -1 *  np.sqrt(2)
        else:
            weight = 1

        grid_graph.add_edges_from([
            ((x, y), (x + 1, y + 1))
            for x in range(self.HEIGHT)
            for y in range(self.WIDTH)
        ] + [
            ((x + 1, y), (x, y + 1))
            for x in range(self.HEIGHT)
            for y in range(self.WIDTH)
        ], weight = weight)

        return grid_graph