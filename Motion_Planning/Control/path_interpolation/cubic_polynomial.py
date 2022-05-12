"""
Cubic Polynomial Path
For Parking Problem / Dubins Curves
"""
import numpy as np
from bisect import bisect_left
from path_interpolation import PathInterpolation
from path_interpolation.parameters import *

# Function to generate quintic polynomial
def generate_cubic_polynomial(
    start_t, start_x, start_v, end_t, end_x, end_v
):
    """
    Function generate a cubic polynomial(degree = 3) 
    given boundary conditions of a differential motion
    model

    Inputs
    ------
    start_t : Start time
    start_x : Start position
    start_v : Start velocity

    end_t : End time
    end_x : End position
    end_v : End velocity

    Outputs
    -------
    polynomial : np.polynomial class object
    """
    # We solve an Ax = b linear equation
    # First Order: a3 x^3 + a2 x^2 + a1 x^1 + a0 1
    # Second Order: a3 3x^2 + a2 2x + a1 1
    first_order_coefficients = np.array([1, 1, 1, 1])
    second_order_coefficients = np.array([3, 2, 1, 0])

    terms = np.array([3, 2, 1])
    power_terms_start = np.power(start_t * np.ones(3), terms)
    power_terms_start = np.append(power_terms_start, [1])
    power_terms_end = np.power(end_t * np.ones(3), terms)
    power_terms_end = np.append(power_terms_end, [1])

    A = np.array([
        np.multiply(first_order_coefficients, power_terms_start), 
        np.multiply(second_order_coefficients, np.append(power_terms_start[1:], [0])), 
        np.multiply(first_order_coefficients, power_terms_end), 
        np.multiply(second_order_coefficients, np.append(power_terms_end[1:], [0])), 
    ])

    b = [start_x, start_v, end_x, end_v]

    coefficients = np.linalg.solve(A, b)

    polynomial = np.poly1d(coefficients)
    return polynomial


class CubicPolynomial:
    """
    Cubic Polynomial
    """
    def __init__(self, x, y, s, initial_yaw, final_yaw, skip_points):
        """ Initialization function """
        self.s = s

        self.x = x; self.y = y
        self.sx = []; self.sy = []

        for index in range(len(x) - 1):
            self.sx.append(self._generate_cubic_spline(x, index))
            self.sy.append(self._generate_cubic_spline(y, index))

        self._point_skip = skip_points
        self._generate_initial_final_paths(initial_yaw, final_yaw)

    def _generate_cubic_spline(self, p, index):
        """ Private function to generate cubic spline for parameter p """
        initial_p = p[index]
        initial_s = self.s[index]

        try:
            final_p = p[index + 1]
            final_s = self.s[index + 1]
            initial_ds = self.s[index + 1] - self.s[index]
            initial_dp = p[index + 1] - p[index]
        except IndexError:
            final_p = p[index]
            final_s = self.s[index]
            initial_ds = 1
            initial_dp = 0

        try:
            final_ds = self.s[index + 2] - self.s[index + 1]
            final_dp = p[index + 2] - p[index + 1]
        except IndexError:
            final_ds = 1
            final_dp = 0

        return generate_cubic_polynomial(initial_s, initial_p, initial_dp / initial_ds, 
                                         final_s, final_p, final_dp / final_ds)

    def calculate_position(self, s):
        """ Function to calculate x, y from parameter s """
        # Taking an epsilon error(not exactly called an error) into account
        if (s >= self.s[0] - 0.5) and (s <= self.s[-1] + 0.5):
            s_i = self._search_index(s, self.s)
            if s_i == -1:
                return None, None

            elif s_i < self._point_skip - 1:
                return self.inital_path_x(s), self.inital_path_y(s)

            elif s_i - len(self.s) > - self._point_skip + 1:
                return self.final_path_x(s), self.final_path_y(s)

            else:
                return self.sx[s_i](s), self.sy[s_i](s)
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
            sx = self.sx[s_i]; sy = self.sy[s_i]
            dx = np.polyder(sx)(s)
            dy = np.polyder(sy)(s)

        yaw = np.arctan2(dy, dx)
        return yaw

    def calculate_curvature(self, s):
        """ Function to calculate curvature from parameter s """
        s_i = self._search_index(s, self.s)

        if s_i < self._point_skip:
            dx = np.polyder(self.inital_path_x)(s)
            ddx = np.polyder(self.inital_path_x, 2)(s)
            dy = np.polyder(self.inital_path_y)(s)
            ddy = np.polyder(self.inital_path_y, 2)(s)

        elif s_i - len(self.s) > - self._point_skip:
            dx = np.polyder(self.final_path_x)(s)
            ddx = np.polyder(self.final_path_x, 2)(s)
            dy = np.polyder(self.final_path_y)(s)
            ddy = np.polyder(self.final_path_y, 2)(s)

        else:
            sx = self.sx[s_i]; sy = self.sy[s_i]
            dx = np.polyder(sx)(s)
            ddx = np.polyder(sx, 2)(s)
            dy = np.polyder(sy)(s)
            ddy = np.polyder(sy, 2)(s)

        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3/2))
        return k

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


class CubicInterpolation(PathInterpolation):
    """ Cubic Spline Interpolation """
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

        spline = CubicPolynomial(x[::5], y[::5], s[::5], 
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