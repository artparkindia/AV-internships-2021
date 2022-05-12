"""
Cost and Heuristic functions

Reference: http://theory.stanford.edu/~amitp/GameProgramming/Variations.html
"""
import numpy as np

# Cost functions
class CostFunction:
    """
    Cost Function class
    """
    @staticmethod
    def cost_function(node_1, node_2, goal_cost):
        """ Template cost function """
        return 0

    @staticmethod
    def static_weight_cost_function(node_1, node_2, current_cost, heuristic, weight):
        """ Static Weight cost function """
        cost = current_cost + weight * heuristic(node_1, node_2)
        return cost

    @staticmethod
    def dynamic_weight_cost_function(node_1, node_2, current_cost, heuristic, weight):
        """ Dynamic Weight cost function """
        h = heuristic(node_1, node_2)
        cost = 0

        if current_cost < (2 * weight - 1) * h:
            cost = current_cost / (2 * weight - 1) + h
        else:
            cost = (current_cost + h) / weight

        return cost


# Heuristic Functions
class HeuristicFunction:
    """ 
    Heuristic Function
    """
    @staticmethod
    def heuristic(node_1, node_2):
        """ Template Function """
        return 0

    @staticmethod
    def euclidean_distance_heuristic(node_1, node_2):
        """ Euclidean Distance heuristic function """
        value = np.hypot(node_1.x - node_2.x, node_1.y - node_2.y)
        return value

    @staticmethod
    def manhattan_distance_heuristic(node_1, node_2):
        """ Manhattan Distance heuristic function """
        value = abs(node_1.x - node_2.x) + abs(node_1.y - node_2.y)
        return value