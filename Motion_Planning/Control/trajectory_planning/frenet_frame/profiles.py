"""
To note the various velocity profiles of the algorithm
"""
import numpy as np
from trajectory_planning import Profile

class LongitudinalProfile(Profile):
    """
    Longitudinal Velocity(ds_dt) profile
    """
    def __init__(self):
        self.title = "Longitudinal Velocity(ds_dt)"
        self.x = []
        self.y = []
    
    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)

class LateralProfile(Profile):
    """
    Lateral Velocity(dd_dt) profile
    """
    def __init__(self):
        self.title = "Lateral Velocity(dd_dt)"
        self.x = []
        self.y = []
    
    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)

class JerkProfile(Profile):
    """
    Longitudinal Jerk (d3s_dt3) profile
    """
    def __init__(self):
        self.title = "Longitudinal Jerk(d3s_dt3)"
        self.x = []
        self.y = []
    
    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)