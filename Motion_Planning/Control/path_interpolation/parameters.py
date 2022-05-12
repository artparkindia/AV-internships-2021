"""
Parameters for Interpolation methods
"""
from path_interpolation import *
import numpy as np

class InitialYawParameter(Parameter):
    """
    Module for the initial yaw parameter
    """
    def __init__(self, interpolator):
        # Initialize basic type and title
        self.interpolator = interpolator
        self.type = 'slider'
        self.title = "INITIAL YAW"

        self.min_value = 0.0
        self.max_value = 2 * np.pi
        self.value = np.pi
        self.step = 0.1

        # Set default value
        self.interpolator.initial_yaw = np.pi

    def set_value(self, value):
        """ Function to set value """
        self.interpolator.initial_yaw = value

class FinalYawParameter(Parameter):
    """
    Module for the final yaw parameter
    """
    def __init__(self, interpolator):
        # Initialize basic type and title
        self.interpolator = interpolator
        self.type = 'slider'
        self.title = "FINAL YAW"

        self.min_value = 0.0
        self.max_value = 2 * np.pi
        self.value = np.pi
        self.step = 0.1

        # Set default value
        self.interpolator.final_yaw = np.pi

    def set_value(self, value):
        """ Function to set value """
        self.interpolator.final_yaw = value

class SkipPointParameter(Parameter):
    """
    Module for the skip point parameter
    """
    def __init__(self, interpolator):
        # Initialize basic type and title
        self.interpolator = interpolator
        self.type = 'slider'
        self.title = "INITIAL/FINAL POINTS TO SKIP"

        self.min_value = 0
        self.max_value = 20
        self.value = 5
        self.step = 1

        # Set default value
        self.interpolator.skip_points = 5

    def set_value(self, value):
        """ Function to set value """
        self.interpolator.skip_points = value