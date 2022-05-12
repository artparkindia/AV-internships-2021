"""
Optimal Trajectory Generation using Frenet Frames

Link to Paper: https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame

Reference: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py

"""

from trajectory_planning import TrajectoryPlanning, Trajectory
from utils.polynomials import *
from copy import deepcopy
import numpy as np

from .lateral import *
from .longitudnal import *
from .parameters import *
from .profiles import *
from .noise import *

class FrenetTrajectory:
    """
    Frenet Trajectory class

    Each Trajectory Point is defined by:
    t: Point of time
    s: Longitudnal Coordinate
    c: Curvature of path
    ds_dt: First Derivative of s
    d2s_dt2: Second Derivative of s
    d3s_dt3: Third Derivate of s
    d: Lateral Coordinate
    dd_dt: First Derivative of d
    d2d_dt2: Second Derivative of d
    d3d_dt3: Third Derivate of d
    dd_dt: First Derivative of d wrt s
    d2d_dt2: Second Derivative of d wrt s
    d3d_dt3: Third Derivate of d wrt s

    Cd: Lateral cost value
    Ct: Longitudnal cost value for Following, Merging and Stopping
    Cv: Longitudnal cost value for Velocity Keeping
    Cf: Total cost value
    """
    def __init__(self):
        self.t = []
        self.s = []
        self.c = []
        self.ds_dt = []
        self.d2s_dt2 = []
        self.d3s_dt3 = []
        self.d = []
        self.dd_dt = []
        self.d2d_dt2 = []
        self.d3d_dt3 = []
        self.dd_ds = []
        self.d2d_ds2 = []
        self.d3d_ds3 = []

        self.Cd = 0
        self.Ct = 0
        self.Cv = 0
        self.Cf = 0

class FrenetFrame(TrajectoryPlanning):
    """
    Frenet Frame method for Optimal Trajectory Generation

    Inputs
    ------
    following_merging(bool): Frenet Frame method for following and merging
    stopping(bool): Frenet Frame method for stopping
    velocity_keeping(bool): Frenet Frame method for velocity keeping

    Parameters
    ----------
    MAX_SPEED: Maximum allowed speed
    MAX_ACCELERATION: Maximum allowed acceleration
    MAX_CURVATURE: Maximum allowed curvature
    MAX_ROAD_WIDTH: Maximum allowed road width
    MAX_T: Maximum time to fulfill a trajectory
    MIN_T: Minimum time to fulfill a trajectory
    ROBOT_RADIUS: Robot radius
    THRESHOLD_DISTANCE: Time Gap Law distance constant
    THRESHOLD_TIME: Time Gap Law time constant

    TARGET_SPEED: Target speed
    TARGET_DISTANCE: Target distance in lateral axis

    DROAD_WIDTH: Sampling constant for road width
    DT: Sampling constant for time
    DTARGET_SPEED: Sampling constant for target speed
    DTARGET_DISTANCE: Sampling constant for target distance
    SAMPLE: Sampling constant

    K_J: Weight constant for square of jerk terms
    K_T: Weight constant for time
    K_D: Weight constant for squared target distance
    K_LAT: Weight constant for Lateral Trajectory
    K_LON: Weight constant for Longitudnal Trajectory

    Methods
    -------
    plan_trajectory: Function to plan a trajectory
    set_initial_conditions: Function to set initial conditions
    for a trajectory

    Assumptions
    -----------
    s and d refer to the longitudnal and lateral coordinates 
    respectively
    """
    def __init__(self, following = False, merging=False, stopping=False, velocity_keeping=True):
        # Initialize conditions
        self.following = following
        self.merging = merging
        self.stopping = stopping
        self.velocity_keeping = velocity_keeping
        
        # Initialize constants
        self.MAX_SPEED = 50
        self.MAX_ACCELERATION = 2
        self.MAX_CURVATURE = 1
        self.MAX_ROAD_WIDTH = 7
        self.MAX_T = 15
        self.MIN_T = 14
        self.THRESHOLD_SPEED = 10
        self.ROBOT_RADIUS = 2
        self.THRESHOLD_DISTANCE = 10
        self.THRESHOLD_TIME = 1

        # Target Constants
        self.TARGET_SPEED = 30
        self.TARGET_DISTANCE = 0

        # Sampling constants
        self.DROAD_WIDTH = 1
        self.DT = 0.2
        self.DTARGET_SPEED = 5
        self.DTARGET_DISTANCE = 5
        self.SAMPLE = 1

        # Weight constants
        self.K_J = 0.1
        self.K_T = 0.1
        self.K_D = 1
        self.K_LAT = 1
        self.K_LON = 1

        self.NOISE = no_noise

        # Initial conditions
        self.set_initial_conditions()

        self.parameters = [
            MAXTParameter(self),
            MINTParameter(self),
            DTParameter(self),
            TargetSpeedParameter(self),
            InitialSpeedParameter(self),
            InitialOffsetParameter(self),
            NoiseModelParameter(self)
        ]

        self.time_index = 0
        self.profiles = [
            LongitudinalProfile(),
            LateralProfile(),
            JerkProfile()
        ]

    def plan_trajectory(self, path, obstacles, others=False, *args):
        """
        Function to plan a trajectory from path

        Inputs
        ------
        path: numpy.polynomial class object
        start_x: start x coordinate
        end_x: end x coordinate

        Outputs
        -------
        optimal_path: Tuple of Trajectory and FrenetTrajectory class objects
        """
        # Calculate valid Frenet Frame Paths
        frenet_paths = self._generate_frenet_paths()
        trajectory_paths = self._generate_global_paths(frenet_paths, path)
        combined_paths = self._generate_optimal_paths(frenet_paths, trajectory_paths, obstacles)

        # Find and return the minimum cost path
        minimum_cost = float("inf")
        optimal_path = None
        for frenet_path, trajectory_path in combined_paths:
            if minimum_cost >= frenet_path.Cf:
                minimum_cost = frenet_path.Cf
                optimal_path = (trajectory_path, frenet_path)

        # Update profiles
        self.profiles[0].add(self.time_index, optimal_path[1].ds_dt[0])
        self.profiles[1].add(self.time_index, optimal_path[1].dd_dt[0])
        self.profiles[2].add(self.time_index, optimal_path[1].d3s_dt3[0])
        self.time_index += 1
        
        if others:
            return optimal_path, trajectory_paths
        else:
            return optimal_path

    def set_initial_conditions_path(self, algorithm_path, index = 0, *args):
        """
        Function to set initial conditions for trajectory
        from algorithm path
        """
        # new_ds_dt, new_dd_dt = self.NOISE(algorithm_path.ds_dt[index], algorithm_path.dd_dt[index])

        self.s = algorithm_path.s[index]
        self.ds_dt = algorithm_path.ds_dt[index]
        self.d2s_dt2 = algorithm_path.d2s_dt2[index]
        self.d = algorithm_path.d[index]
        self.dd_dt = algorithm_path.dd_dt[index]
        self.d2d_dt2 = algorithm_path.d2d_dt2[index]

        self.t = 0


    def set_initial_conditions(self, s = 0, ds_dt = 0, d2s_dt2 = 0, 
                                     d = 0, dd_dt = 0, d2d_dt2 = 0, t = 0,
                                     final_s = None, obs_a_s = None, 
                                     obs_a_ds_dt = None, obs_b_s = None,
                                     obs_b_ds_dt = None):
        """
        Function to set initial conditions for a trajectory

        Parameters
        ----------
        s: Starting Lateral Distance
        ds_dt: Starting Lateral speed
        d2s_dt2: Starting Lateral acceleration

        d: Starting Longitudnal distance
        dd_dt: Starting Longitudal speed
        d2d_dt2: Starting Longitudnal acceleration

        t: Time elapsed

        path: numpy.polynomial class object
        start_x: start x coordinate
        end_x: end x coordinate

        Returns
        -------
        None
        """
        self.s = s
        self.ds_dt = ds_dt
        self.d2s_dt2 = d2s_dt2
        self.d = d
        self.dd_dt = dd_dt
        self.d2d_dt2 = d2d_dt2

        self.t = t

        # For Stopping
        if final_s is not None:
            self.TARGET_DISTANCE = final_s
        
        # For Following
        if obs_a_s is not None:
            self.s_leading_vehicle = obs_a_s
            self.ds_dt_leading_vehicle = obs_a_ds_dt

        # For Merging
        if obs_b_s is not None:
            self.s_a = obs_a_s; self.s_b = obs_b_s
            self.ds_dt_a = obs_a_ds_dt; self.ds_dt_b = obs_b_ds_dt

    def _generate_global_paths(self, frenet_paths, path):
        """
        Private function to generate global coordinate path
        from frenet coordinate path

        Inputs
        ------
        frenet_paths: List of FrenetTrajectory objects
        path: numpy.polynomial class object

        Returns
        -------
        trajectory: List of Trajectory objects
        """
        trajectory_paths = []

        # Iterate over each frenet path
        for frenet_path in frenet_paths:
            trajectory = Trajectory()
            # Calculate x and y
            for index in range(len(frenet_path.t)):
                x, y = path.calculate_position(frenet_path.s[index])
                
                if x is None:
                    break

                yaw = self.NOISE(path.calculate_yaw(frenet_path.s[index]))
                d = frenet_path.d[index]

                new_x = x + d * np.cos(yaw + np.pi / 2) 
                new_y = y + d * np.sin(yaw + np.pi / 2)

                new_x, new_y = self.NOISE(new_x), self.NOISE(new_y)

                trajectory.x.append(new_x)
                trajectory.y.append(new_y)

            # Calculate yaw, velocity and delta s
            delta_s = []
            for index in range(len(trajectory.x) - 1):
                dx = trajectory.x[index + 1] - trajectory.x[index]
                dy = trajectory.y[index + 1] - trajectory.y[index]
                trajectory.v_x.append(dx / self.DT)
                trajectory.v_y.append(dy / self.DT)
                trajectory.yaw.append(np.arctan2(dy, dx))
                delta_s.append(np.hypot(dx, dy))

            trajectory.v_x.append(trajectory.v_x[-1])
            trajectory.v_y.append(trajectory.v_y[-1])
            trajectory.yaw.append(trajectory.yaw[-1])
            delta_s.append(delta_s[-1])

            # Calculate acceleration
            for index in range(len(trajectory.v_x) - 1):
                dx = trajectory.v_x[index + 1] - trajectory.v_x[index]
                dy = trajectory.v_y[index + 1] - trajectory.v_y[index]
                trajectory.a_x.append(dx / self.DT)
                trajectory.a_y.append(dy / self.DT)

            trajectory.a_x.append(trajectory.a_x[-1])
            trajectory.a_y.append(trajectory.a_y[-1])

            # Calculate curvature
            for index in range(len(trajectory.yaw) - 1):
                frenet_path.c.append((trajectory.yaw[index + 1] - trajectory.yaw[index]) / delta_s[index])

            trajectory_paths.append(trajectory)

        return trajectory_paths

    def _generate_frenet_paths(self):
        """
        Private function to generate frenet paths

        Inputs
        ------
        None

        Outputs
        -------
        frenet_paths: List of Trajectory class objects
        """
        frenet_paths = []

        # Loop over each condition of final t, d and s
        for final_t in np.arange(self.MIN_T, self.MAX_T, self.DT):
            for final_d in np.arange(-self.MAX_ROAD_WIDTH, self.MAX_ROAD_WIDTH + self.DROAD_WIDTH, 
                           self.DROAD_WIDTH):
                trajectory = FrenetTrajectory()
                
                # Generate Lateral Movement
                if self.ds_dt >= self.THRESHOLD_SPEED:
                    generate_lateral_movement_high(self, final_t, final_d, trajectory)

                # Velocity Keeping
                if self.velocity_keeping:
                    for final_ds_dt in np.arange(
                        self.TARGET_SPEED - self.DTARGET_SPEED * self.SAMPLE,
                        self.TARGET_SPEED + self.DTARGET_SPEED * self.SAMPLE, 
                        self.DTARGET_SPEED):

                        final_trajectory = deepcopy(trajectory)

                        # Generate Longitudnal Movement
                        generate_longitudnal_movement_b(self, final_t, final_ds_dt, final_trajectory)

                        # Generate Lateral Movement
                        if self.ds_dt < self.THRESHOLD_SPEED:
                            generate_lateral_movement_low(self, final_d, final_trajectory)
                            calculate_cost_function_b(self, final_t, final_trajectory, final_trajectory.s[-1])
                        else:
                            # Calculate the costs
                            calculate_cost_function_b(self, final_t, final_trajectory)

                        frenet_paths.append(final_trajectory)

                # Following
                if self.following and final_d != -self.MAX_ROAD_WIDTH:
                    for (s, ds_dt) in zip(self.s_leading_vehicle, self.ds_dt_leading_vehicle):
                        for target_s in np.arange(s - self.DTARGET_DISTANCE * self.SAMPLE, 
                            s + self.DTARGET_DISTANCE * self.SAMPLE, self.DTARGET_DISTANCE):
                            final_trajectory = deepcopy(trajectory)
                            
                            # Assuming constant velocity obstacle
                            final_s = target_s - (self.THRESHOLD_DISTANCE + self.THRESHOLD_TIME * ds_dt)
                            final_ds_dt = ds_dt
                            final_d2s_dt2 = 0

                            # Generate Longitudnal Movement
                            generate_longitudnal_movement_a(self, final_t, final_s, final_ds_dt,
                                                            final_d2s_dt2, final_trajectory)

                            # Generate Lateral Movement
                            if self.ds_dt < self.THRESHOLD_SPEED:
                                generate_lateral_movement_low(self, final_d, final_trajectory)
                                calculate_cost_function_a(self, final_t, final_trajectory, final_trajectory.s[-1])
                            else:
                                # Calculate the costs
                                calculate_cost_function_a(self, final_t, final_trajectory)

                            frenet_paths.append(final_trajectory)

                # Merging
                if self.merging:
                    for (s_a, s_b, ds_dt_a, ds_dt_b) in zip(self.s_a, self.s_b, self.ds_dt_a, self.ds_dt_b):
                        final_trajectory = deepcopy(trajectory)

                        # Assuming constant velocity obstacles
                        final_s = (s_a + s_b) / 2
                        final_ds_dt = (ds_dt_a + ds_dt_b) / 2
                        final_d2s_dt2 = 0

                        # Generate Longitudnal Movement
                        generate_longitudnal_movement_a(self, final_t, final_s, final_ds_dt,
                                                        final_d2s_dt2, final_trajectory)

                        # Generate Lateral Movement
                        if self.ds_dt < self.THRESHOLD_SPEED:
                            generate_lateral_movement_low(self, final_d, final_trajectory)
                            calculate_cost_function_a(self, final_t, final_trajectory, final_trajectory.s[-1])
                        else:
                            # Calculate the costs
                            calculate_cost_function_a(self, final_t, final_trajectory)

                        frenet_paths.append(final_trajectory)


                # Stopping
                if self.stopping:
                    final_trajectory = deepcopy(trajectory)

                    final_s = self.TARGET_DISTANCE

                    # Generate Longitudnal Movement
                    generate_longitudnal_movement_a(self, final_t, final_s, 0, 0, final_trajectory)

                    # Generate Lateral Movement
                    if self.ds_dt < self.THRESHOLD_SPEED:
                        generate_lateral_movement_low(self, final_d, final_trajectory)
                        calculate_cost_function_a(self, final_t, final_trajectory, final_trajectory.s[-1])
                    else:
                        # Calculate the costs
                        calculate_cost_function_a(self, final_t, final_trajectory)

                    frenet_paths.append(final_trajectory)

        return frenet_paths

    def _check_collisions(self, trajectory, obstacles):
        """
        Private function to check for collisions in trajectory

        Inputs
        ------
        trajectory: Trajectory Object for the trajectory
        obstacles: 2d list of obstacle locations

        Outputs
        -------
        collision: Whether collision is taking place or not 
        """
        if obstacles == None:
            return True

        if hasattr(obstacles, 'check_collision'):
            return obstacles.check_collision(trajectory)

        # Obstacle Avoidance
        for index in range(len(obstacles.x)):
            distance = [((x_i - obstacles.x[index]) ** 2 + (y_i - obstacles.y[index]) ** 2)
                        for (x_i, y_i) in zip(trajectory.x, trajectory.y)]
            collision = any([distance_i <= self.ROBOT_RADIUS ** 2 for distance_i in distance])
            if collision:
                return False
        
        return True

    def _generate_optimal_paths(self, frenet_paths, trajectory_paths, obstacles):
        """
        Private generator function to generate optimal paths
        given all the generated paths

        Inputs
        ------
        frenet_paths: List of FrenetTrajectory objects
        trajectory_paths: List of Trajectory objects
        obstacles: List of 2d obstacle locations

        Returns
        -------
        combined_paths: List of tuple of satisfying 
        frenet_path and trajectory path
        """
        combined_paths = []

        count_v = 0; count_a = 0; count_c = 0; count_co = 0; count_s = 0
        for frenet_path, trajectory_path in zip(frenet_paths, trajectory_paths):
            if any([speed > self.MAX_SPEED for speed in frenet_path.ds_dt]):
                count_v += 1
                continue
            elif any([abs(acceleration) > self.MAX_ACCELERATION \
                for acceleration in frenet_path.d2s_dt2]):
                count_a += 1
                continue
            elif any([abs(curvature) > self.MAX_CURVATURE \
                for curvature in frenet_path.c]):
                count_c += 1
                continue
            elif not self._check_collisions(trajectory_path, obstacles):
                count_co += 1
                continue

            count_s += 1
            combined_paths.append((frenet_path, trajectory_path))

        speed = "HIGH"
        if self.ds_dt < self.THRESHOLD_SPEED:
            speed = "LOW"

        print(f"Total Trajectories: {count_v + count_a + count_c + count_s + count_co}")
        print(f"{speed} Speed Trajectory")
        print(f"Rejected due to velocity constraint: {count_v}")
        print(f"Rejected due to acceleration constraint: {count_a}")
        print(f"Rejected due to curvature constraint: {count_c}")
        print(f"Rejected due to collision constraint: {count_co}")
        print(f"Selection done from: {count_s}")
        print("")
        return combined_paths
        
