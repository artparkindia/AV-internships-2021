"""
Main simulation script
"""

from simulation.path import CircularPath
from simulation.simulation import Simulation
from simulation.obstacles import Obstacles
from trajectory_planning.frenet_frame import FrenetFrame
import numpy as np
import pygame

WIDTH = 200
HEIGHT = 200

def change_obstacles(obstacles, current_s, path):
    """ Function to change the current obstacles """
    index_1 = 0; index_2 = 0; index_change = 0

    for index in range(len(obstacles.obstacles_x)):
        if index % 3 == 2:
            continue

        s = path.calculate_arclength(obstacles.obstacles_x[index], obstacles.obstacles_y[index], obstacles.obstacles_radii[index])

        if index_change == 0 and current_s < s:
            index_1 = index; index_change = 1
        elif index_change == 1 and current_s < s:
            index_2 = index; index_change = 3
        elif index_change == 3:
            break

    obs_x = np.array([obstacles.obstacles_x[index_1], obstacles.obstacles_x[index_2]])
    obs_y = np.array([obstacles.obstacles_y[index_1], obstacles.obstacles_y[index_2]])
    obs_radii = np.array([obstacles.obstacles_radii[index_1], obstacles.obstacles_radii[index_2]])
    obs_omega = np.array([obstacles.obstacles_omega[index_1], obstacles.obstacles_omega[index_2]])

    return obs_x, obs_y, obs_radii, obs_omega

if __name__ == "__main__":
    path = CircularPath(-WIDTH/2, 0, 300, 50)
    
    obstacles = Obstacles()
    obstacles.intialize_many_circular_obstacles_dynamic(path)

    simulation = Simulation(WIDTH, HEIGHT)

    # Trajectory planning
    trajectory = FrenetFrame(following = True, velocity_keeping = True)
    trajectory.K_LAT = 1
    trajectory.K_LON = 1
    trajectory.MAX_T = 10
    trajectory.MIN_T = 9
    trajectory.MAX_ROAD_WIDTH = 50
    trajectory.DROAD_WIDTH = 50
    trajectory.MAX_CURVATURE = 5
    trajectory.MAX_ACCELERATION = 10
    trajectory.TARGET_SPEED = 30 / 3.6
    trajectory.MAX_SPEED = 100 / 3.6
    trajectory.THRESHOLD_DISTANCE = 10
    trajectory.THRESHOLD_TIME = 0.01


    obs_x, obs_y, obs_radii, obs_omega = change_obstacles(obstacles, 0, path)
    trajectory.set_initial_conditions(
        0,
        30 / 3.6,
        0,
        0,
        0,
        0,
        0,
        None,
        list(path.calculate_arclength(obs_x, obs_y, obs_radii)),
        list(obs_radii * obs_omega)
    )

    running = True

    # N steps follower
    N = 1; index = N

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
    
        if index == N:
            try:
                (trajectory_path, frenet_path), others = trajectory.plan_trajectory(path, obstacles, True)
                print(f"Current speed: {frenet_path.ds_dt[1]}")
            except TypeError:
                print("Could not find any valid trajectories")

            index = 0
        else:
            obstacles.move(path.center_x, path.center_y, trajectory.DT)
            simulation.update(path, trajectory_path, obstacles, others)
            
            # trajectory_path.t.pop(0)
            trajectory_path.x.pop(0); trajectory_path.y.pop(0); trajectory_path.yaw.pop(0)
            trajectory_path.v_x.pop(0); trajectory_path.v_y.pop(0)
            trajectory_path.a_x.pop(0); trajectory_path.a_y.pop(0)
            frenet_path.s.pop(0); frenet_path.d.pop(0)
            frenet_path.ds_dt.pop(0); frenet_path.dd_dt.pop(0)
            frenet_path.d2s_dt2.pop(0); frenet_path.d2d_dt2.pop(0)

            index += 1

        obs_x, obs_y, obs_radii, obs_omega = change_obstacles(obstacles, frenet_path.s[1], path)
        trajectory.set_initial_conditions(
            frenet_path.s[1],
            frenet_path.ds_dt[1],
            frenet_path.d2s_dt2[1],
            frenet_path.d[1],
            frenet_path.dd_dt[1],
            frenet_path.d2d_dt2[1],
            0,
            None,
            list(path.calculate_arclength(obs_x, obs_y, obs_radii)),
            list(obs_radii * obs_omega)
        )