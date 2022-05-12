"""
Parameters for Frenet Frame Algorithm
"""
from trajectory_planning import *
from .noise import *

class MAXTParameter(Parameter):
    """
    Module for the maximum planning time
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "MAXIMUM PLANNING TIME"

        self.min_value = 2
        self.max_value = 30
        self.value = 20
        self.step = 1

        # Set default value
        self.algorithm.MAX_T = 20

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.MAX_T = value

class MINTParameter(Parameter):
    """
    Module for the minimum planning time
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "MINIMUM PLANNING TIME"

        self.min_value = 1
        self.max_value = 30
        self.value = 18
        self.step = 1

        # Set default value
        self.algorithm.MIN_T = 18

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.MIN_T = value

class DTParameter(Parameter):
    """
    Module for the difference in pose
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "POSE DIFFERENCE TIME"

        self.min_value = 0.1
        self.max_value = 1.0
        self.value = 0.5
        self.step = 0.1

        # Set default value
        self.algorithm.DT = 0.5

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.DT = value

class TargetSpeedParameter(Parameter):
    """
    Module for Target Speed
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "TARGET SPEED"

        self.min_value = 10
        self.max_value = int(self.algorithm.MAX_SPEED)
        self.value = 20
        self.step = 1

        # Set default value
        self.algorithm.TARGET_SPEED = 30

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.TARGET_SPEED = value

class InitialSpeedParameter(Parameter):
    """
    Module for Initial Speed
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "INITIAL SPEED"

        self.min_value = -10
        self.max_value = int(self.algorithm.MAX_SPEED)
        self.value = 10
        self.step = 1

        # Set default value
        self.algorithm.ds_dt = 20

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.ds_dt = value

class InitialOffsetParameter(Parameter):
    """
    Module for Initial Offset
    """
    def __init__(self, algorithm):
        # Initialize basic type and title
        self.algorithm = algorithm
        self.type = 'slider'
        self.title = "INITIAL OFFSET"

        self.min_value = -50
        self.max_value = 50
        self.value = 0
        self.step = 10

        # Set default value
        self.algorithm.d = 0

    def set_value(self, value):
        """ Function to set value """
        self.algorithm.d = value

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