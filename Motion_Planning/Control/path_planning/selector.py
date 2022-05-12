"""
Path Planning Algorithm selector
"""
from .a_star import AStar
from .a_star_nx import AStarX

path_planning_algorithms = ["astar", "astar_nx"]

def select_path_planner(algorithm):
    if algorithm == 'astar':
        return AStar()
    elif algorithm == "astar_nx":
        return AStarX()