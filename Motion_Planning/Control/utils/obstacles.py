"""
Python module for obstacles and their behaviour

- Static Obstacles
- Dynamic Obstacles
"""

import numpy as np

class Obstacles:
    """
    Class for obstacles and their behaviour
    """
    def __init__(self, obstacle_coordinates):
        """ Initialization function """
        self.x, self.y = self._separate_x_y(obstacle_coordinates)
        self.vel_x, self.vel_y = np.zeros(len(self.x)), np.zeros(len(self.y))

    def move(self, DT):
        """
        Function to move the obstacles

        Inputs
        ------
        DT: time difference to move through

        Outputs
        -------
        None
        """
        self.x = self.x + self.vel_x * DT
        self.y = self.y + self.vel_y * DT

    def set_velocity(self, obstacle_velocities):
        """ 
        Function to set velocity of obstacles 
        
        Inputs
        ------
        obstacle_velocities: 2d list of velocities

        Outputs
        -------
        None
        """
        self.vel_x, self.vel_y = self._separate_x_y(obstacle_velocities)


    def _separate_x_y(self, coordinates):
        """
        Private function to separate x and y

        Input
        -----
        coordinates: List of 2D tuples | List of 2D list

        Output
        ------
        x, y: Two numpy arrays of floats
        """
        # Convert to numpy array
        np_coordinates = np.array(coordinates)

        return np_coordinates[:,0], np_coordinates[:,1]