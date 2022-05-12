"""
Main script
"""

from path_interpolation.vandermonde import VanderMonde
from trajectory_planning.frenet_frame import FrenetFrame
from trajectory_planning.frenet_frame.noise import *
from utils.plotting import *
import numpy as np

if __name__ == "__main__":
    points = [(0, 0), (10, -6), (30, 5), (75, 6.5), (120, 0)]
    obstacles = None
    nice_test_points_1 = [(0, 0), (5, -6), (10, 5), (15, 6.5), (20, 0)]
    interpolation = VanderMonde()
    path = interpolation.interpolate(points)

    # Trajectory planning
    trajectory = FrenetFrame(stopping = True)
    trajectory.TARGET_SPEED = 4
    trajectory.MAX_CURVATURE = 20
    trajectory.NOISE = gaussian_noise

    trajectory.set_initial_conditions(
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        path.s[-1] - 0.2
    )

    simulation_loops = 500
    total_time = (simulation_loops / trajectory.DT) / 100
    time_elapsed = 0

    trajectory.MAX_T = total_time
    trajectory.MIN_T = total_time - 1

    for sim in range(simulation_loops):
        try:
            trajectory_path, frenet_path = trajectory.plan_trajectory(path, obstacles)
        except TypeError:
            raise Exception("Could not find any valid trajectories")

        if np.hypot(trajectory_path.x[1] - points[-1][0], trajectory_path.y[1] - points[-1][1]) <= 1.0:
            print("Goal")
            break
        
        plot_simulation(path, trajectory_path, obstacles, sim)
        print(f"Current speed: {frenet_path.ds_dt[1]}")

        time_elapsed += trajectory.DT    
        trajectory.set_initial_conditions_path(frenet_path, index = 1)