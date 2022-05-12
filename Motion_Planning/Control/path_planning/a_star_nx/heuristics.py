"""
Cost and Heuristic functions

Reference: http://theory.stanford.edu/~amitp/GameProgramming/Variations.html
"""
import numpy as np

# NetworkX Functions
class HeuristicXFunction:
    """ 
    Heuristic Function for NetworkX
    """
    @staticmethod
    def heuristic(node_1, node_2):
        """ Template Function """
        return 0

    @staticmethod
    def euclidean_distance_heuristic(node_1, node_2):
        """ Euclidean Distance heuristic function """
        (x1, y1) = node_1
        (x2, y2) = node_2
        value = np.hypot(x1 - x2, y1 - y2)
        return value

    @staticmethod
    def manhattan_distance_heuristic(node_1, node_2):
        """ Manhattan Distance heuristic function """
        (x1, y1) = node_1
        (x2, y2) = node_2
        value = abs(x1 - x2) + abs(y1 - y2)
        return value