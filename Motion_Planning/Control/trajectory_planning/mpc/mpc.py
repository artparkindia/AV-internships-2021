from trajectory_planning.mpc.parameters import DTParameter, MotionModelParameter, NoiseModelParameter, TParameter, TargetSpeedParameter, WeightParameter
from trajectory_planning import TrajectoryPlanning, Trajectory
import numpy as np
import math

from scipy.optimize import minimize

from .model import *
from .utils import *
from .profiles import *
from .noise import *

class State:
    """
    Vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega = omega

class StatePath:
    """
    Vehicle State path
    """

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.omega = []
        self.acc = []
        self.jerk = []

class ModelPredictiveControl(TrajectoryPlanning):
    """
    Model Predictive Control Trajectory Planner

    Parameters
    ----------
    NX: Number of dependent variables
    NU: Number of independent variables
    T: Number of time steps to look ahead
    DT: Duration of a time step
    DL: Tick for generating path

    TARGET_SPEED: Speed to target
    WEIGHTS: Weights for [x, y, v, yaw, acc, omega, jerk]

    MAX_STEER: Maximum steering angle
    MIN_STEER: Minimum steering angle
    MAX_SPEED: Maximum possible speed
    MIN_SPEED: Minimum possible speed
    MAX_ACCELERATION: Maximum possible acceleration
    MIN_ACCELERATION: Minimum possible acceleration
    MAX_D_STEER: Maximum angle to steer from current position
    MAX_CURVATURE: Maximum possible curvature
    MIN_CURVATURE: Minimum possible curvature

    N_IND_SEARCH: Number of indices to look ahead while getting immediate reference path
    MAX_ITER: Maximum number of times MPC runs to get optimal solution in single call
    ITER_STOP: Stop the loop earlier if there is not much change in the solution

    COST: Cost model for the algorithm
    """
    def  __init__(self):
        """
        Initialization function
        """
        # Initialize Constants
        self.NX = 4
        self.NU = 2
        self.T = 10 
        self.DT = 0.1 
        self.DL = 1

        # Constants
        self.TARGET_SPEED = 5 #
        self.WEIGHTS = [1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5] #

        # Bounds
        # Advanced Options....
        self.MAX_STEER = np.deg2rad(35)
        self.MIN_STEER = np.deg2rad(-35)
        self.MAX_SPEED = 30.0 #
        self.MIN_SPEED = -30.0 #
        self.MAX_ACCELERATION = 10
        self.MIN_ACCELERATION = -10
        self.MAX_D_STEER = np.deg2rad(10)
        self.MAX_CURVATURE = 10
        self.MIN_CURVATURE = -10

        # MPC Parameters
        self.N_IND_SEARCH = 10
        self.MAX_ITER = 5
        self.ITER_STOP = 0.01

        # External Constants
        self.MODEL = BicycleModel(self)
        self.NOISE = no_noise

        self.set_initial_conditions()

        # Parameters and Profiles
        self.parameters = [
            TParameter(self),
            DTParameter(self),
            TargetSpeedParameter(self),
            MotionModelParameter(self),
            NoiseModelParameter(self),
            WeightParameter(0, "X WEIGHT", 1.0, self),
            WeightParameter(1, "Y WEIGHT", 1.0, self),
            WeightParameter(2, "VELOCITY WEIGHT", 0.5, self),
            WeightParameter(3, "YAW WEIGHT", 0.5, self),
            WeightParameter(4, "ACCELERATION WEIGHT", 0.5, self),
            WeightParameter(5, "OMEGA WEIGHT", 0.5, self),
            WeightParameter(6, "JERK WEIGHT", 0.5, self),
        ]

        self.time_index = 0
        self.profiles = [
            VelocityProfile(),
            OmegaProfile(),
            JerkProfile()
        ]

    def set_initial_conditions(self):
        """
        Function to set initial conditions
        """
        self._reference_path = None
        self._v_state = None
        self._omega_state = None
        self._pindex = 0

    def set_initial_conditions_path(self, algorithm_path, index = 1, *args):
        """
        Function to set initial conditions from path
        """
        self.x = algorithm_path.x[index]
        self.y = algorithm_path.y[index]
        self.yaw = algorithm_path.yaw[index]
        self.v = algorithm_path.v[index]
        self.omega = algorithm_path.omega[index]

    def plan_trajectory(self, path, *args):
        """
        Function plan the trajectory given path
        """
        # If this is the first iteration
        if self._reference_path is None:
            x, y, yaw = self._calculate_course(path)
            
            speed_profile = self._calculate_speed_profile(x, y, yaw, self.TARGET_SPEED)
            t = int(self.TARGET_SPEED / self.MIN_ACCELERATION * self.DT)
            for index in range(len(speed_profile) - t, len(speed_profile) - 1):
                speed_profile[index] = speed_profile[index-1] + self.MIN_ACCELERATION * self.DT

            self._reference_path = np.array([x, y, speed_profile, yaw])
            self._reference_path[3] = smooth_yaw(self._reference_path[3])
            
            state = StatePath()
            state.x.append(self._reference_path[0][0])
            state.y.append(self._reference_path[1][0])
            state.yaw.append(self._reference_path[3][0])
            state.v.append(0.0); state.omega.append(0.0)

            self.set_initial_conditions_path(state, index = 0)

        # Solve the optimization
        self._pindex = self._solve()

        # Get and return global trajectories
        trajectory_path, state_path = self._generate_global_paths()

        # Update profiles
        self.profiles[0].add(self.time_index, state_path.v[0])
        self.profiles[1].add(self.time_index, state_path.omega[0])
        self.profiles[2].add(self.time_index, state_path.jerk[0])
        self.time_index += 1


        return trajectory_path, state_path

    def _solve(self):
        """
        ref is a numpy matrix of
            cx  : course x position list
            cy  : course y position list
            cv  : target velocity
            cyaw: course yaw position list
        """

        # get ref trajectory 
        target_ind = self.calc_nearest_index()
        xref, dref = self.calc_ref_trajectory(target_ind, self.v)

        #initial guess i.e. reference states
        if self._v_state is None or self._omega_state is None:
            self._v_state = np.zeros(self.T)
            self._omega_state = np.zeros(self.T)
        
        t = self.T

        x0 = np.concatenate((self._v_state, self._omega_state))
        
        #get constraints and bounds
        constraints,bounds = self.constraints()
        x = x0
        #iterative mpc (updaing initial guess at every iteration)
        for i in range(self.MAX_ITER):
            poa, pod = self._v_state[:], self._omega_state[:]
            res = minimize(self.MODEL.calculate_cost, x, args=(xref), method="SLSQP", bounds=bounds, constraints=constraints)
            if res.success:
                x = res.x
                self._v_state = x[: t]
                self._omega_state = x[t: 2*t]
                du = np.absolute(self._v_state - poa).sum() + np.absolute(self._omega_state - pod).sum()  # calc u change value
                if du <= self.ITER_STOP:
                    break
            else:
                print('res failed')
        if np.all(np.concatenate((self._v_state,self._omega_state))==x0):
            return target_ind

        return target_ind

    def _generate_global_paths(self):
        """
        Function to generate global paths from the calculated
        path
        """
        trajectory = Trajectory()
        state = State()
        state_path = StatePath()

        x = self.x; y = self.y; yaw = self.yaw
        state.x = x; state.y = y; state.yaw = yaw
        state.v, state.omega = self.NOISE(self._v_state[0], self._omega_state[0])

        for v, omega in zip(self._v_state, self._omega_state):
            # Generate State Path
            state_path.x.append(state.x); state_path.y.append(state.y)
            state_path.v.append(state.v); state_path.omega.append(state.omega)
            state_path.yaw.append(state.yaw)

            # Generate Trajectory Path
            trajectory.x.append(state.x)
            trajectory.y.append(state.y)

            # For next iteration
            state.x = state.x + state.v * np.cos(state.yaw) * self.DT
            state.y = state.y + state.v * np.sin(state.yaw) * self.DT
            state.yaw = self.MODEL.calculate_yaw(state)
            state.v, state.omega = self.NOISE(v, omega) 

        state_path.acc = list(np.array(state_path.v[1:]) - np.array(state_path.v[:-1]))
        state_path.jerk = list(np.array(state_path.acc[1:]) - np.array(state_path.acc[:-1]))

        return trajectory, state_path


    def _calculate_course(self, path, delta_s = 0.5):
        """
        Private function to calculate discrete course from path object
        """
        s = list(np.arange(0, path.s[-1], delta_s))

        x, y, yaw = [], [], []
        for s_i in s:
            ix, iy = path.calculate_position(s_i)
            x.append(ix); y.append(iy)
            yaw.append(path.calculate_yaw(s_i))

        return x, y, yaw

    def _calculate_speed_profile(self, cx, cy, cyaw, target_speed):
        """
        Private function to calculate the speed profile
        """
        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0

        return speed_profile

    def constraints(self):
        """
            cons = [{'type':'eq', 'fun': con},
                    {'type':'ineq', 'fun': con_real}]
        """
        t = self.T

        constraints = []
        # constraint for omega of steering angle to be lesser than self.max_d_steer
        # cons = {'type':'ineq', 'fun': lambda x: self.max_d_steer - np.abs(x[t]-state.omega)}
        # constraints.append(cons)

        # constraints for min and max acceleration
        cons = {'type':'ineq', 'fun': lambda x: self.MAX_ACCELERATION * self.DT - (x[0]-self.v)}
        constraints.append(cons)
        cons = {'type':'ineq', 'fun': lambda x:  ( x[0]-self.v ) - self.MIN_ACCELERATION * self.DT}
        constraints.append(cons)

        bounds = [(None,None) for _ in range(2*t)]

        for v,omega in zip(range(t), range(t , 2*t)):
            #set upper and lower bounds for velocity and steering angle
            bounds[v] = (self.MIN_SPEED, self.MAX_SPEED)
            bounds[omega] = (self.MIN_STEER, self.MAX_STEER)
            
            if(omega > t):
                # constraint for omega of steering angle to be lesser than self.max_d_steer
                cons = {'type':'ineq', 'fun': lambda x: self.MAX_D_STEER * self.DT - np.abs(x[omega]-x[omega-1])}
                constraints.append(cons)

                # constraints for min and max acceleration
                cons = {'type':'ineq', 'fun': lambda x: self.MAX_ACCELERATION * self.DT - (x[v]-x[v-1])}
                constraints.append(cons)
                cons = {'type':'ineq', 'fun': lambda x:  ( x[v]-x[v-1] ) - self.MIN_ACCELERATION * self.DT}
                constraints.append(cons)

            #constraints for curvature
            # cons = {'type':'ineq', 'fun': lambda x: (self.max_curvature - ( np.abs(np.rad2deg(x[omega])/x[v]))) if x[v]!=0 else -1}
            # cons = {'type':'ineq', 'fun': lambda x: (self.max_curvature - ( np.rad2deg(x[omega])/x[v])) if x[v]!=0 else -1}
            cons = {'type':'ineq', 'fun': lambda x: np.abs(x[v]) - np.abs(np.rad2deg(x[omega])) }
            constraints.append(cons)
            # cons = {'type':'ineq', 'fun': lambda x: ((np.rad2deg(x[omega])/x[v]) - self.min_curvature) if x[v]!=0 else -1}
            # constraints.append(cons)

        
        return constraints,bounds

    def calc_ref_trajectory(self, ind, v):
        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = self._reference_path[0].size

        xref[0, 0] = self._reference_path[0][ind]
        xref[1, 0] = self._reference_path[1][ind]
        xref[2, 0] = self._reference_path[2][ind]
        xref[3, 0] = self._reference_path[3][ind]
        # dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(self.T + 1):
            travel += abs(v) * self.DT
            dind = int(round(travel / 1.0))

            if (ind + dind) < ncourse:
                xref[0, i] = self._reference_path[0][ind + dind]
                xref[1, i] = self._reference_path[1][ind + dind]
                xref[2, i] = self._reference_path[2][ind + dind]
                xref[3, i] = self._reference_path[3][ind + dind]
            else:
                xref[0, i] = self._reference_path[0][ncourse - 1]
                xref[1, i] = self._reference_path[1][ncourse - 1]
                xref[2, i] = self._reference_path[2][ncourse - 1]
                xref[3, i] = self._reference_path[3][ncourse - 1]

        return xref, dref
 
    def calc_nearest_index(self):
        pind = self._pindex

        dx = np.subtract(self.x,self._reference_path[0,pind:(pind + self.N_IND_SEARCH)])
        dy = np.subtract(self.y,self._reference_path[1,pind:(pind + self.N_IND_SEARCH)])

        d = dx**2 + dy**2

        ind = np.argmin(d)
        ind = ind + pind

        return ind

    def calc_cte(self, ref, ind):
        return np.sqrt((self.x-ref[0][ind])**2 + (self.y-ref[1][ind])**2)