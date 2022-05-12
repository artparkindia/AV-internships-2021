"""
Module for Path Planning

Input(A Map along with the start and end positions)
                    |
                    V
                Path Planning
                    |
                    V
            Output(Points to follow)
"""

# Base class for other classes to follow
import numpy as np


class PathPlanning:
    """
    Base class for other path planning 
    algorithms

    Methods
    -------
    set_parameters(): Function to set parameters
    plan_path(): Function to plan the path
    """
    def set_parameters(self, parameters):
        """ 
        Function to set parameters of a particular algorithm
        """
        
        raise NotImplementedError

    def plan_path(self, map, start, end):
        """
        Function to plan a path on a given map with
        start and end positions
        """

        raise NotImplementedError


# Base class for a Node used in Path Planning
class Node:
    """
    Base class for a Node

    Each node is defined by:
    x: x coordinate of the node
    y: y coordinate of the node
    cost: cost to reach to the node
    parent: parent node
    """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.cost = 0
        self.parent_index = -1

# Base class for parameters of an algorithm
class Parameter:
    """
    Base class for Parameters
    """
    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selection = ["A", "B", "C"]
        self.title = 'Weight'

    def set_value(self, value):
        """ Function to set value """

        raise NotImplementedError

# Statistics for Path Planning Algorithms
class Statistics:
    """
    Statistics Template to compare various path planning algorithms
    """
    def __init__(self):
        self.title = "Statistic 1"
        self.value = 0

    def set_value(self, value):
        self.value = value

    def get_value(self):
        return self.value

# Motion Models
# dx, dy, cost
MOTION_8WAY = [
    [1, 0, 1],
    [0, 1, 1],
    [-1, 0, 1],
    [0, -1, 1],
    [-1, -1, np.sqrt(2)],
    [-1, 1, np.sqrt(2)],
    [1, -1, np.sqrt(2)],
    [1, 1, np.sqrt(2)]
]

MOTION_8WAYEQUAL = [
    [1, 0, 1],
    [0, 1, 1],
    [-1, 0, 1],
    [0, -1, 1],
    [-1, -1, 1],
    [-1, 1, 1],
    [1, -1, 1],
    [1, 1, 1]
]

MOTION_8WAYLESS = [
    [1, 0, 1],
    [0, 1, 1],
    [-1, 0, 1],
    [0, -1, 1],
    [-1, -1, - np.sqrt(2)],
    [-1, 1, - np.sqrt(2)],
    [1, -1, - np.sqrt(2)],
    [1, 1, - np.sqrt(2)]
]

MOTION_4WAY = [
    [1, 0, 1],
    [0, 1, 1],
    [-1, 0, 1],
    [0, -1, 1]
]