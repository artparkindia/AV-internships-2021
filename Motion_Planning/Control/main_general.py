"""
Main file to start general simulation
"""
from trajectory_planning.mpc.noise import *
from general_simulation.simulation import Simulation
from general_simulation.obstacles import Obstacles
from path_planning.a_star import AStar
from path_planning.a_star_nx import AStarX
from trajectory_planning.frenet_frame import FrenetFrame
from trajectory_planning.mpc import ModelPredictiveControl
from path_interpolation.linear_spline import LinearInterpolation
import json

MAP = 'arena'

def _interpret_instructions(map, instructions = 'instructions.json'):
    """ Function to interpret the json instructions """
    data = None
    with open(instructions) as ins:
        data = json.load(ins)

    return data[map]["start"], data[map]["end"]

if __name__ == "__main__":
    simulation = Simulation(MAP)
    obstacles = Obstacles(MAP)

    start_pos, end_pos = _interpret_instructions(MAP)
    plan_path = AStar()
    points = plan_path.plan_path(obstacles, start_pos, end_pos)

    simulation.save_path(points, 'outputs/path_planning')

    interpolation = LinearInterpolation()
    path = interpolation.interpolate(points)

    simulation.save_interpolation(path, 'outputs/path_interpolation')

    trajectory = FrenetFrame()
    trajectory.MAX_T = 20
    trajectory.MIN_T = 18
    trajectory.DT = 0.5
    trajectory.MAX_CURVATURE = 0.1
    trajectory.MAX_ROAD_WIDTH = 100
    trajectory.DROAD_WIDTH = 20
    trajectory.MAX_SPEED = 50
    trajectory.TARGET_SPEED = 30
    trajectory.MAX_ACCELERATION = 10
    trajectory.THRESHOLD_SPEED = 20

    trajectory.K_J = 0.1
    trajectory.K_T = 0.1
    trajectory.K_D = 1
    trajectory.K_LAT = 1
    trajectory.K_LON = 1

    trajectory.set_initial_conditions(
        0,
        30,
        0,
        20,
        0,
        0,
        0,
        path.s[-1] - 0.2
    )


    simulation.initialize_plots()
    while True:
        trajectory_path, frenet_path = trajectory.plan_trajectory(path, obstacles)

        completed = simulation.update(path, trajectory_path, None)

        if completed:
            simulation.save_trajectory(f'outputs/trajectory_planning')
            break

        print(f"Current speed: {frenet_path.ds_dt[1]}")
        trajectory.set_initial_conditions_path(frenet_path, index = 1)
        # trajectory.set_initial_conditions(
        #     frenet_path.s[1],
        #     frenet_path.ds_dt[1],
        #     frenet_path.d2s_dt2[1],
        #     frenet_path.d[1],
        #     frenet_path.dd_dt[1],
        #     frenet_path.d2d_dt2[1]
        # )