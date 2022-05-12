"""
Main simulation script
"""

from simulation.path import CircularPath
from simulation.simulation import Simulation
from simulation.obstacles import Obstacles
from trajectory_planning.frenet_frame import FrenetFrame
import pygame

WIDTH = 720
HEIGHT = 540

if __name__ == "__main__":
    path = CircularPath(WIDTH / 2, HEIGHT / 2, 150, 50)
    
    obstacles = Obstacles()
    obstacles.intialize_circular_obstacles_static(path)

    simulation = Simulation(WIDTH, HEIGHT)

    # Trajectory planning
    trajectory = FrenetFrame()
    trajectory.MAX_T = 20
    trajectory.MIN_T = 19
    trajectory.MAX_ROAD_WIDTH = 60
    trajectory.DROAD_WIDTH = 15
    trajectory.MAX_CURVATURE = 10

    trajectory.set_initial_conditions(
        0,
        10 / 3.6,
        0,
        0,
        0,
        0,
        0
    )

    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
    
        try:
            trajectory_path, frenet_path = trajectory.plan_trajectory(path, obstacles)
        except TypeError:
            raise Exception("Could not find any valid trajectories")
        
        print(f"Current speed: {frenet_path.ds_dt[1]}")

        trajectory.set_initial_conditions(
            frenet_path.s[1],
            frenet_path.ds_dt[1],
            frenet_path.d2s_dt2[1],
            frenet_path.d[1],
            frenet_path.dd_dt[1],
            frenet_path.d2d_dt2[1],
        )

        obstacles.move(path.center_x, path.center_y, trajectory.DT)
        simulation.update(path, trajectory_path, obstacles)