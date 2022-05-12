"""
Python module for the path to follow

- CircularPath
"""
import numpy as np

class CircularPath:
    """
    Module for a circular path 
    """
    def __init__(self, center_x, center_y, radius, width, lanes=3):
        """ Initialization function """
        self.radius = radius
        self.width = width
        self.lanes = lanes
        self.center_x = center_x
        self.center_y = center_y
    
    def calculate_position(self, s):
        """ Function to calculate x and y from s """
        # Assuming s = 0 at at X=r and Y=0
        # Positive direction anticlockwise
        theta_radians = s / self.radius
        if abs(theta_radians) > 2 * np.pi:
            return None, None
            
        x = self.radius * np.cos(theta_radians) + self.center_x
        y = self.radius * np.sin(theta_radians) + self.center_y
        return x, y

    def calculate_yaw(self, s):
        """ Function to calculate yaw from s """
        theta = s / self.radius
        dx = -1 * np.sin(theta); dy = np.cos(theta)
        yaw = np.arctan2(dy, dx)

        return yaw

    def calculate_curvature(self, s):
        """ Function to calculate curvature from arclength s """
        theta = s / self.radius
        dx = -1 * np.sin(theta); dy = np.cos(theta)
        ddx = -1 * np.cos(theta) / self.radius
        ddy = -1 * np.sin(theta) / self.radius

        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3/2))
        return k

    def calculate_arclength(self, x, y, radius):
        """ Function to calculate arclength s from x and y """
        theta = np.arctan2(y - self.center_y, x - self.center_x)
        try:
            theta[theta < 0] = theta[theta < 0] + 2 * np.pi
            s = radius * theta
        except TypeError:
            if theta < 0:
                theta += 2 * np.pi
            s = radius * theta

        return s