"""
Path Interpolation using Linear Spline Method
"""

from typing import final
import numpy as np
from .cubic_polynomial import generate_cubic_polynomial
from bisect import bisect_left
from path_interpolation import PathInterpolation
from path_interpolation.parameters import *

class LinearSpline:
    """ Linear Spline Function """
    def __init__(self, x, y, s, initial_yaw, final_yaw, skip_points):
        """ Initialization function """
        self.s = s
        self.x = x
        self.y = y

        self._point_skip = skip_points
        self._generate_initial_final_paths(initial_yaw, final_yaw)

    def calculate_position(self, s):
        """ Function to calculate x, y from parameter s """
        if (s >= self.s[0] - 0.5) and (s <= self.s[-1] + 0.5):
            s_i = self._search_index(s, self.s)
            if s_i == -1:
                return None, None
            
            elif s_i < self._point_skip - 1:
                return self.inital_path_x(s), self.inital_path_y(s)

            elif s_i - len(self.s) > - self._point_skip + 1:
                return self.final_path_x(s), self.final_path_y(s)

            else:
                theta = self.calculate_yaw(s)
                ds = s - self.s[s_i]
                return self.x[s_i] + ds * np.cos(theta), self.y[s_i] + ds * np.sin(theta)
        else:
            return None, None

    def calculate_yaw(self, s):
        """ Function to calculate yaw from parameter s """
        s_i = self._search_index(s, self.s)

        if s_i < self._point_skip:
            dx = np.polyder(self.inital_path_x)(s)
            dy = np.polyder(self.inital_path_y)(s)

        elif s_i - len(self.s) > - self._point_skip:
            dx = np.polyder(self.final_path_x)(s)
            dy = np.polyder(self.final_path_y)(s)

        else:
            try:
                dx = self.x[s_i + 1] - self.x[s_i]
                dy = self.y[s_i + 1] - self.y[s_i]
            except IndexError:
                dx = self.x[s_i] - self.x[s_i - 1]
                dy = self.y[s_i] - self.y[s_i - 1]

        yaw = np.arctan2(dy, dx)
        return yaw

    def _search_index(self, s, x):
        """ Private function to search for an index """
        if s == 0:
            return 0

        index = bisect_left(x, s)
        if index:
            return index - 1
        else:
            return -1

    def _generate_initial_final_paths(self, initial_yaw, final_yaw):
        """ Private function to generate the initial and final paths """

        self.inital_path_x = generate_cubic_polynomial(
            self.s[0], self.x[0], np.cos(initial_yaw), 
            self.s[self._point_skip], self.x[self._point_skip], 
            (self.x[self._point_skip + 1] - self.x[self._point_skip]) / (self.s[self._point_skip + 1] - self.s[self._point_skip]) 
        )
        self.inital_path_y = generate_cubic_polynomial(
            self.s[0], self.y[0], np.sin(initial_yaw), 
            self.s[self._point_skip], self.y[self._point_skip], 
            (self.y[self._point_skip] - self.y[self._point_skip - 1]) / (self.s[self._point_skip] - self.s[self._point_skip - 1]) 
        )

        self.final_path_x = generate_cubic_polynomial(
            self.s[-self._point_skip], self.x[-self._point_skip],
            (self.x[-self._point_skip + 1] - self.x[-self._point_skip]) / (self.s[-self._point_skip + 1] - self.s[-self._point_skip]),
            self.s[-1], self.x[-1], np.cos(final_yaw)
             
        )
        self.final_path_y = generate_cubic_polynomial(
            self.s[-self._point_skip], self.y[-self._point_skip],
            (self.y[-self._point_skip + 1] - self.y[-self._point_skip]) / (self.s[-self._point_skip + 1] - self.s[-self._point_skip]),
            self.s[-1], self.y[-1], np.sin(final_yaw)
        )


class LinearInterpolation(PathInterpolation):
    """ Linear Spline Interpolation """
    def __init__(self):
        self.parameters = [
            InitialYawParameter(self),
            FinalYawParameter(self),
            SkipPointParameter(self)
        ]

    def interpolate(self, coordinates):
        """ Function to interpolate the points(coordinates) """
        # Separate x and y coordinates
        x, y = self._separate_x_y(coordinates)

        # Generate parameteric arclength
        s = self._calculate_parameter(x, y)

        spline = LinearSpline(x, y, s,
                 self.initial_yaw, self.final_yaw, self.skip_points)

        return spline


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

    def _calculate_parameter(self, x, y):
        """
        Private function to calculate parameteric version
        of polynomial

        Input
        -----
        x: numpy array of floats
        y: numpy array of floats

        Output
        ------
        s: parameter of the parametric version
        """
        dx = np.diff(x); dy = np.diff(y)
        ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(ds))
        return s