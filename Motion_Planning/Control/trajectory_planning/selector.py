"""
Trajectory Planning Algorithm selector
"""
from .frenet_frame import FrenetFrame
from .mpc import ModelPredictiveControl

trajectory_planning_algorithms = ["frenet_frame", "mpc"]

def select_trajectory_planner(algorithm):
    if algorithm == 'frenet_frame':
        return FrenetFrame()
    elif algorithm == 'mpc':
        return ModelPredictiveControl()