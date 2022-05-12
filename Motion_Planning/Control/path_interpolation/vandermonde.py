"""
VanderMonde Matrix method for Polynomial
Interpolation

Given n points: (x1, y1), (x2, y2), ..., (xn, yn)
Solve for a general n-1 degree polynomial
c0 + c1x + c2x^2 + c3x^3 + ... + cn-1x^n-1

by solving
Vc = y

V (VanderMonde Matrix) = [
    [1, x1, x1^2, x1^3, ..., x1^n-1],
    [1, x2, x2^2, x2^3, ..., x2^n-1],
    ...
    [1, xn, xn^2, xn^3, ..., xn^n-1]
]

c = [c0, c1, c2, c3, ..., cn-1].T
y = [y1, y2, y3, ..., yn].T
"""

from path_interpolation import PathInterpolation
import numpy as np

class PathPolynomial:
    """
    Pararmetric Polynomial for path
    x and y are determined using parameter arclength

    Inputs
    ------
    polynomial_x: np.poly1d object for polynomial x(s)
    polynomial_y: np.poly1d object for polynomial y(s)
    s: Discrete values of s at points of (x, y)

    Parameters
    ----------
    s: Discrete values of s at points of (x, y)
    sx: np.poly1d object for polynomial x(s)
    sy: np.poly1d object for polynomial y(s)
    """
    def __init__(self, polynomial_x, polynomial_y, s):
        """ Initialization function """
        self.s = s
        self.sx = polynomial_x
        self.sy = polynomial_y

    def calculate_position(self, s):
        """ Function to calculate x, y from parameter s """
        # Taking an epsilon error(not exactly called an error) into account
        if (s >= self.s[0] - 0.5) and (s <= self.s[-1] + 0.5):
            x = self.sx(s); y = self.sy(s)
            return x, y
        else:
            return None, None

    def calculate_yaw(self, s):
        """ Function to calculate yaw from parameter s """
        dx = np.polyder(self.sx)(s)
        dy = np.polyder(self.sy)(s)
        yaw = np.arctan2(dy, dx)
        return yaw

    def calculate_curvature(self, s):
        """ Function to calculate curvature from parameter s """
        dx = np.polyder(self.sx)(s)
        ddx = np.polyder(self.sx, 2)(s)
        dy = np.polyder(self.sy)(s)
        ddy = np.polyder(self.sy, 2)(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3/2))
        return k

class VanderMonde(PathInterpolation):
    """
    VanderMonde matrix method for Polynomial 
    Interpolation
    """

    def interpolate(self, coordinates):
        """
        Function to interpolate a list of coordinates

        Input
        -----
        coordinates: List of 2D tuples | List of 2D list

        Output
        ------
        path: PathPolynomial object from utils

        Assumptions
        -----------
        Assume the number of coordinates be n
        """
        # Separate x and y coordinates
        x, y = self._separate_x_y(coordinates)

        # Generate parameteric arclength
        s = self._calculate_parameter(x, y)

        # Generate the VanderMonde Matrices
        vandermonde_matrix = self._generate_vandermonde_matrix(s)

        # Solve for the coefficients
        coefficients_x = self._solve(vandermonde_matrix, x)
        coefficients_y = self._solve(vandermonde_matrix, y)

        # Generate the n-1 degree polynomial
        polynomial_x = np.poly1d(coefficients_x)
        polynomial_y = np.poly1d(coefficients_y)

        # Generate Path object
        path = PathPolynomial(polynomial_x, polynomial_y, s)

        return path

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

    def _generate_vandermonde_matrix(self, x):
        """
        Private function to generate vandermonde matrix

        Input
        -----
        x : numpy array of floats

        Output
        ------
        matrix : 2d numpy matrix
        """
        # Use numpy function
        matrix = np.vander(x)

        return matrix

    def _solve(self, A, b):
        """
        Private function to solve Ax = b

        Input
        -----
        A : 2d numpy matrix
        b : 1d numpy array

        Output
        ------
        x : 1d numpy array
        """
        # Use numpy function
        x = np.linalg.solve(A, b)

        return x