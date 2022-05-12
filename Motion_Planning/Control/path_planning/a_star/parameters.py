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

class HeuristicFunctionParameter(Parameter):
    """
    Module for the heuristic function parameter
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selections = {
            "Euclidean Distance": HeuristicFunction.euclidean_distance_heuristic, 
            "Manhattan Distance": HeuristicFunction.manhattan_distance_heuristic
        }
        self.title = "HEURISTIC FUNCTION"

        # Set default value
        self.algorithm.HEURISTIC = HeuristicFunction.euclidean_distance_heuristic

    def set_value(self, value):
        """ Function to set value from one of the selections """
        self.algorithm.HEURISTIC = self.selections[value]

class WeightParameter(Parameter):
    """
    Module for the weight parameter
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "WEIGHT"

        self.min_value = 0
        self.max_value = 20
        self.value = 0
        self.step = 1

        # Set default value
        self.algorithm.WEIGHT = 1

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.WEIGHT = value

class CostFunctionParameter(Parameter):
    """
    Module for the cost function parameter
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selections = {
            "Static Cost Function": CostFunction.static_weight_cost_function, 
            "Dynamic Cost Function": CostFunction.dynamic_weight_cost_function
        }
        self.title = "COST FUNCTION"

        # Set default value
        self.algorithm.COST_FUNCTION = CostFunction.static_weight_cost_function

    def set_value(self, value):
        """ Function to set value from one of the selections """
        self.algorithm.COST_FUNCTION = self.selections[value]
