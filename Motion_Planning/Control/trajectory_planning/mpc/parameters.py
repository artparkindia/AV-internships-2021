"""
Parameters for MPC Algorithm
"""
from trajectory_planning import *
from .model import *
from .noise import *

class TParameter(Parameter):
    """
    Module for the planning time
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "PLANNING TIME"

        self.min_value = 1
        self.max_value = 20
        self.value = 10
        self.step = 1

        # Set default value
        self.algorithm.T = 20

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.T = value

class DTParameter(Parameter):
    """
    Module for the difference in state times
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "TIME DIFFERENCE"

        self.min_value = 0.1
        self.max_value = 5.0
        self.value = 0.1
        self.step = 0.1

        # Set default value
        self.algorithm.DT = 0.1

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.DT = value

class TargetSpeedParameter(Parameter):
    """
    Module for the target speed
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "TARGET SPEED"

        self.min_value = 5
        self.max_value = 30
        self.value = 5
        self.step = 5

        # Set default value
        self.algorithm.TARGET_SPEED = 5

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.TARGET_SPEED = value

class MotionModelParameter(Parameter):
    """
    Module for the motion model parameter
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selections = {
            "bicycle": BicycleModel, 
            "unicycle": UnicycleModel
        }
        self.title = "MOTION MODEL"

        # Set default value
        self.algorithm.MODEL = BicycleModel(self.algorithm)

    def set_value(self, value):
        """ Function to set value from one of the selections """
        self.algorithm.MODEL = self.selections[value](self.algorithm)

class NoiseModelParameter(Parameter):
    """
    Module for the noise model parameter
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selections = {
            "no noise": no_noise, 
            "gaussian noise": gaussian_noise
        }
        self.title = "NOISE MODEL"

        # Set default value
        self.algorithm.NOISE = no_noise

    def set_value(self, value):
        """ Function to set value from one of the selections """
        self.algorithm.NOISE = self.selections[value]

class WeightParameter(Parameter):
    """
    Module for the weight list
    """
    def __init__(self, index, title, init_value, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = title
        self.index = index

        self.min_value = 0.0
        self.max_value = 1.0
        self.value = init_value
        self.step = 0.1

        # Set default value
        self.algorithm.WEIGHTS[self.index] = init_value

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.WEIGHTS[self.index] = value

