"""
Noise Model
"""
import numpy as np

def no_noise(s):
    """ No noise model """
    return s

def gaussian_noise(x, dev = 0.1):
    """ Gaussian noise model """
    # s_  = np.random.normal(s, 0.1 * np.abs(s))
    # d_  = np.random.normal(d, 0.1 * np.abs(d))
    x_  = np.random.normal(x, dev)

    return x_