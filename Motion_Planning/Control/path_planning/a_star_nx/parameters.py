"""
Parameters for A Star Algorithm
"""
from path_planning import *
from .heuristics import *

class MotionModelParameter(Parameter):
    """
    Module for the motion model parameter
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selections = {
            "4way": MOTION_4WAY, 
            "8way": MOTION_8WAY, 
            "8way_equal": MOTION_8WAYEQUAL,
            "8way_less": MOTION_8WAYLESS
        }
        self.title = "MOTION MODEL"

        # Set default value
        self.algorithm.MOTION_MODEL = MOTION_8WAYEQUAL

    def set_value(self, value):
        """ Function to set value from one of the selections """
        self.algorithm.MOTION_MODEL = self.selections[value]

class HeuristicXFunctionParameter(Parameter):
    """
    Module for the heuristic function parameter for networkx
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selections = {
            "Euclidean Distance": HeuristicXFunction.euclidean_distance_heuristic, 
            "Manhattan Distance": HeuristicXFunction.manhattan_distance_heuristic
        }
        self.title = "HEURISTIC X FUNCTION"

        # Set default value
        self.algorithm.HEURISTICX = HeuristicXFunction.euclidean_distance_heuristic

    def set_value(self, value):
        """ Function to set value from one of the selections """
        self.algorithm.HEURISTICX = self.selections[value]
