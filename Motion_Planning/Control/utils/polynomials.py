"""
Utility functions related to polynomials
"""

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

def generate_quintic_polynomial(
    start_t, start_x, start_v, start_a, end_t, end_x, end_v, end_a
):
    """
    Function generate a quintic polynomial(degree = 5) 
    given boundary conditions of a differential motion
    model

    Inputs
    ------
    start_t : Start time
    start_x : Start position
    start_v : Start velocity
    start_a : Start acceleration

    end_t : End time
    end_x : End position
    end_v : End velocity
    end_a : End acceleration

    Outputs
    -------
    polynomial : np.polynomial class object
    """
    # We solve an Ax = b linear equation
    # First Order: a5 x^5 + a4 x^4 + a3 x^3 + a2 x^2 + a1 x + a0 1
    # Second Order: 0x^5 + a5 5x^4 + a4 4x^3 + a3 3x^2 + a2 2x + a1 1
    # Third Order: 0x^5 + 0x^4 + a5 20x^3 + a4 12x^2 + a3 6x + a2 2
    first_order_coefficients = np.array([1, 1, 1, 1, 1, 1])
    second_order_coefficients = np.array([5, 4, 3, 2, 1, 0])
    third_order_coefficients = np.array([20, 12, 6, 2, 0, 0])

    terms = np.array([5, 4, 3, 2, 1])
    power_terms_start = np.power(start_t * np.ones(5), terms)
    power_terms_start = np.append(power_terms_start, [1])
    power_terms_end = np.power(end_t * np.ones(5), terms)
    power_terms_end = np.append(power_terms_end, [1])

    A = np.array([
        np.multiply(first_order_coefficients, power_terms_start), 
        np.multiply(second_order_coefficients, np.append(power_terms_start[1:], [0])), 
        np.multiply(third_order_coefficients, np.append(power_terms_start[2:], [0, 0])),
        np.multiply(first_order_coefficients, power_terms_end), 
        np.multiply(second_order_coefficients, np.append(power_terms_end[1:], [0])), 
        np.multiply(third_order_coefficients, np.append(power_terms_end[2:], [0, 0]))
    ])

    b = [start_x, start_v, start_a, end_x, end_v, end_a]

    coefficients = np.linalg.solve(A, b)

    polynomial = np.poly1d(coefficients)
    return polynomial


def generate_quartic_polynomial(
    start_t, start_x, start_v, start_a, end_t, end_v, end_a
):
    """
    Function generate a quintic polynomial(degree = 5) 
    given boundary conditions of a differential motion
    model

    Inputs
    ------
    start_t : Start time
    start_x : Start position
    start_v : Start velocity
    start_a : Start acceleration

    end_t : End time
    end_v : End velocity
    end_a : End acceleration

    Outputs
    -------
    polynomial : np.polynomial class object
    """
    # We solve an Ax = b linear equation
    # First Order: a4 x^4 + a3 x^3 + a2 x^2 + a1 x + a0 1
    # Second Order: 0x^4 + a4 4x^3 + a3 3x^2 + a2 2x + a1 1
    # Third Order: 0x^4 + 0x^3 + a4 12x^2 + a3 6x + a2 2
    first_order_coefficients = np.array([1, 1, 1, 1, 1])
    second_order_coefficients = np.array([4, 3, 2, 1, 0])
    third_order_coefficients = np.array([12, 6, 2, 0, 0])

    terms = np.array([4, 3, 2, 1])
    power_terms_start = np.power(start_t * np.ones(4), terms)
    power_terms_start = np.append(power_terms_start, [1])
    power_terms_end = np.power(end_t * np.ones(4), terms)
    power_terms_end = np.append(power_terms_end, [1])

    A = np.array([
        np.multiply(first_order_coefficients, power_terms_start), 
        np.multiply(second_order_coefficients, np.append(power_terms_start[1:], [0])), 
        np.multiply(third_order_coefficients, np.append(power_terms_start[2:], [0, 0])),
        np.multiply(second_order_coefficients, np.append(power_terms_end[1:], [0])), 
        np.multiply(third_order_coefficients, np.append(power_terms_end[2:], [0, 0]))
    ])

    b = [start_x, start_v, start_a, end_v, end_a]

    coefficients = np.linalg.solve(A, b)

    polynomial = np.poly1d(coefficients)
    return polynomial