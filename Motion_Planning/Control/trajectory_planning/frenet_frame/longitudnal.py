"""
Methods for Generating Longitudnal Coordinate Trajectories

A. Following, Merging and Stopping
B. Velocity Keeping
"""

from utils.polynomials import *
import numpy as np

def generate_longitudnal_movement_a(self, final_t, final_s, 
    final_ds_dt, final_d2s_dt2, trajectory):
    """
    Function to generate longitudnal frenet trajectory
    for subpart a following merging and stopping

    Inputs
    ------
    final_t: boundary condition of final time
    final_s: boundary condition of final s
    final_ds_dt: boundary condition of final ds_dt
    final_d2s_dt2: boundary condition of final d2s_dt2
    trajectory: FrenetTrajectory class object

    Outputs
    -------
    None
    """
    polynomial = generate_quintic_polynomial(
        self.t, self.s, self.ds_dt, self.d2s_dt2, 
        final_t, final_s, final_ds_dt, final_d2s_dt2
    )

    _update_trajectory_object(self, final_t, trajectory, polynomial)

def generate_longitudnal_movement_b(self, final_t, final_ds_dt, trajectory):
    """
    Function to generate longitudnal frenet trajectory
    for subpart b velocity keeping

    Inputs
    ------
    final_t: boundary condition of final time
    final_ds_dt: boundary condition of final ds_dt
    trajectory: FrenetTrajectory class object

    Outputs
    -------
    None
    """
    polynomial = generate_quartic_polynomial(
        self.t, self.s, self.ds_dt, self.d2s_dt2, final_t, final_ds_dt, 0
    )

    _update_trajectory_object(self, final_t, trajectory, polynomial)


def calculate_cost_function_a(self, final_t, trajectory, final_s = None):
    """
    Function to calculate cost based on lateral
    and longitudnal information for longitudnal movement
    subpart a: Following Merging and Stopping

    Inputs
    ------
    final_t: boundary condition of final time
    trajectory: FrenetTrajectory class object

    Outputs
    -------
    None
    """

    # Square of jerk terms
    Jp = sum(np.power(trajectory.d3d_dt3, 2))
    Js = sum(np.power(trajectory.d3s_dt3, 2))
    Jss = sum(np.power(trajectory.d3d_ds3, 2))

    # Square of difference from target speed
    ds = (self.TARGET_DISTANCE - trajectory.s[-1]) ** 2
    d2 = trajectory.d[-1] ** 2

    # Final cost calculations
    if final_s is not None:
        trajectory.Cd = self.K_J * Jss + self.K_T * (final_s - self.s) + self.K_D * d2
    else:
        trajectory.Cd = self.K_J * Jp + self.K_T * (final_t - self.t) + self.K_D * d2
    trajectory.Ct = self.K_J * Js + self.K_T * (final_t - self.t) + self.K_D * ds
    trajectory.Cf = self.K_LAT * trajectory.Cd + self.K_LON * trajectory.Ct

def calculate_cost_function_b(self, final_t, trajectory, final_s = None):
    """
    Function to calculate cost based on lateral
    and longitudnal information for longitudnal movement
    subpart b: Velocity Keeping

    Inputs
    ------
    final_t: boundary condition of final time
    trajectory: FrenetTrajectory class object

    Outputs
    -------
    None
    """

    # Square of jerk terms
    Jp = sum(np.power(trajectory.d3d_dt3, 2))
    Js = sum(np.power(trajectory.d3s_dt3, 2))
    Jss = sum(np.power(trajectory.d3d_ds3, 2))

    # Square of difference from target speed
    ds2 = (self.TARGET_SPEED - trajectory.ds_dt[-1]) ** 2
    d2 = trajectory.d[-1] ** 2

    # Final cost calculations
    if final_s is not None:
        trajectory.Cd = self.K_J * Jss + self.K_T * (final_s - self.s) + self.K_D * d2
    else:
        trajectory.Cd = self.K_J * Jp + self.K_T * (final_t - self.t) + self.K_D * d2
    trajectory.Cv = self.K_J * Js + self.K_T * (final_t - self.t) + self.K_D * ds2
    trajectory.Cf = self.K_LAT * trajectory.Cd + self.K_LON * trajectory.Cv


def _update_trajectory_object(self, final_t, trajectory, polynomial):
    """
    Private Function to update trajectory object

    Inputs
    ------
    final_t: boundary condition of final time
    trajectory: FrenetTrajectory class object
    polynomial: Polynomial to update trajectory from

    Returns
    -------
    None
    """
    time_slots = [t for t in np.arange(self.t, final_t, self.DT)]
    trajectory.t = time_slots
    trajectory.s = [polynomial(t) for t in time_slots]
    trajectory.ds_dt = [np.polyder(polynomial)(t) for t in time_slots]
    trajectory.d2s_dt2 = [np.polyder(polynomial, 2)(t) for t in time_slots] 
    trajectory.d3s_dt3 = [np.polyder(polynomial, 3)(t) for t in time_slots]