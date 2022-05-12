"""
To note the various velocity profiles of the algorithm
"""
import numpy as np
from trajectory_planning import Profile

class VelocityProfile(Profile):
    """
    Velocity profile
    """
    def __init__(self):
        self.title = "Velocity"
        self.x = []
        self.y = []
    
    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)

class OmegaProfile(Profile):
    """
    Omega profile
    """
    def __init__(self):
        self.title = "Omega"
        self.x = []
        self.y = []
    
    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)

class JerkProfile(Profile):
    """
    Jerk Profile
    """
    def __init__(self):
        self.title = "Jerk"
        self.x = []
        self.y = []
    
    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)