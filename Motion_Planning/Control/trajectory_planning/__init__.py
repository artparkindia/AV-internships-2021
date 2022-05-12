"""
Module for Trajectory Planning

Input(Path(Mathematically a function defined in a range) to follow)
                                |
                                V
                        Trajectory Planning
                                |
                                V
            Output(Trajectory to follow to follow the path)
"""

# Define a base class for other classes to follow
class TrajectoryPlanning:
    """
    Base class for other Trajectory Planning
    algorithms

    Methods
    -------
    plan_trajectory() : Function to plan a trajectory
    set_initial_conditions_algorithm() : Function to set initial
    conditions using algorithm specific state variable
    """

    def plan_trajectory(self, path, *args):
        """
        Function to plan a trajectory on a given path
        """

        raise NotImplementedError

    def set_initial_conditions_path(self, path, *args):
        """
        Function to set initial conditions using path
        """

        raise NotImplementedError

# Define a base class for Trajectory
class Trajectory:
    """
    Base class for Trajectory

    Each Trajectory Point is defined by:
    t : Point of time
    x : x coordinate
    y : y coordinate
    yaw : direction
    v_x : Velocity along x
    v_y : Velocity along y
    a_x : Acceleration along x
    a_y : Acceleration along y
    """
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.yaw = []
        self.v_x = []
        self.v_y = []
        self.a_x = []
        self.a_y = []



# Base class for parameters of an algorithm
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

# Base class for velocity profiles
class Profile:
    """
    Base class for Profile
    """
    def __init__(self):
        self.title = "Variable"
        self.x = []
        self.y = []

    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)