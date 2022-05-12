"""
Module for Path Interpolation

Input(List of (X, Y) points to follow)
                |
                V
          Path Interpolation
                |
                V
Output(Function passing through those (X, Y) points)
"""

# Define a base class for other classes to follow
class PathInterpolation:
    """
    Base class for other Path Interpolation
    classes

    Methods
    -------
    interpolate() : Function to interpolate
    """

    def interpolate(self, coordinates):
        """
        Function to interpolate a list of coordinates
        """
        raise NotImplementedError

# Base class for parameters for an interpolator
class Parameter:
    """
    Base class for Parameters
    """
    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.type = 'selectbox'
        self.selection = ["A", "B", "C"]
        self.title = 'Weight'

    def set_value(self, value):
        """ Function to set value """

        raise NotImplementedError