"""
Noise Model
"""
import numpy as np

def no_noise(v, omega):
    """ No noise model """
    return v, omega

def gaussian_noise(v, omega):
    """ Gaussian noise model """
    # v_  = np.random.normal(v, 0.1*np.abs(v))
    # omega_  = np.random.normal(omega, 0.1*np.abs(omega))
    v_  = np.random.normal(v, 0.01)
    omega_  = np.random.normal(omega, 0.01)
    return v_, omega_