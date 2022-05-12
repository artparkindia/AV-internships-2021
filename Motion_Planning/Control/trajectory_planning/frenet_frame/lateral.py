"""
Methods for Generating Lateral Coordinate Trajectories

High Speed Trajectories
Low Speed Trajectories
"""
from utils.polynomials import generate_quintic_polynomial
import numpy as np

def generate_lateral_movement_high(self, final_t, final_d, trajectory):
    """
    Function to generate lateral frenet trajectory
    for high speeds

    Inputs
    ------
    final_d: boundary condition of final d
    final_t: boundary condition of final time
    trajectory: FrenetTrajectory class object

    Outputs
    -------
    None
    """
    polynomial = generate_quintic_polynomial(
        self.t, self.d, self.dd_dt, self.d2d_dt2, final_t, final_d, 0, 0
    )

    time_slots = [t for t in np.arange(self.t, final_t, self.DT)]
    trajectory.t = time_slots
    trajectory.d = [polynomial(t) for t in time_slots]
    trajectory.dd_dt = [np.polyder(polynomial)(t) for t in time_slots]
    trajectory.d2d_dt2 = [np.polyder(polynomial, 2)(t) for t in time_slots] 
    trajectory.d3d_dt3 = [np.polyder(polynomial, 3)(t) for t in time_slots]

def generate_lateral_movement_low(self, final_d, trajectory):
    """
    Function to generate lateral frenet trajectory
    for low speeds

    Inputs
    ------
    final_s: boundary condition of final s
    final_t: boundary condition of final time
    trajectory: FrenetTrajectory class object

    Outputs
    -------
    None
    """
    # Assuming the calculation takes place after s has been calculated
    try:
        dd_ds = self.dd_dt * (1 / self.ds_dt)
        d2t_ds2 = -1 * self.d2s_dt2 / (self.ds_dt) ** 3
        d2d_ds2 = (self.d2d_dt2 * (1 / self.ds_dt) ** 2) + (self.dd_dt * d2t_ds2)
    except ZeroDivisionError:
        dd_ds = 0
        d2t_ds2 = 0
        d2d_ds2 = 0

    polynomial = generate_quintic_polynomial(
        self.s, self.d, dd_ds, d2d_ds2, trajectory.s[-1], final_d, 0, 0
    )

    trajectory.d = [polynomial(s) for s in trajectory.s]
    trajectory.dd_ds = [np.polyder(polynomial)(s) for s in trajectory.s]
    trajectory.d2d_ds2 = [np.polyder(polynomial, 2)(s) for s in trajectory.s] 
    trajectory.d3d_ds3 = [np.polyder(polynomial, 3)(s) for s in trajectory.s]

    for index in range(len(trajectory.d)):
        dd_ds = trajectory.dd_ds[index]; ds_dt = trajectory.ds_dt[index]
        d2d_ds2 = trajectory.d2d_ds2[index]; d2s_dt2 = trajectory.d2s_dt2[index]
        d3d_ds3 = trajectory.d3d_ds3[index]; d3s_dt3 = trajectory.d3s_dt3[index]

        trajectory.dd_dt.append(dd_ds * ds_dt)
        trajectory.d2d_dt2.append(d2d_ds2 * (ds_dt ** 2) + dd_ds * d2s_dt2)
        trajectory.d3d_dt3.append(d3d_ds3 * (ds_dt ** 3) + 3 * d2d_ds2 * ds_dt * d2s_dt2 + dd_ds * d3s_dt3)