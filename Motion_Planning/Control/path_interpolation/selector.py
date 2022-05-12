"""
Path Interpolation Algorithm selector
"""
from .vandermonde import VanderMonde
from .linear_spline import LinearInterpolation
from .cubic_polynomial import CubicInterpolation

path_interpolation_methods = ["linear", "cubic"]

def select_path_interpolator(method):
    if method == 'linear':
        return LinearInterpolation()
    
    elif method == "cubic":
        return CubicInterpolation()