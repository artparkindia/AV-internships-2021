"""
Statistics to compare Path Planning Algorithms
"""
from path_planning import Statistics

class PathLength(Statistics):
    """
    Path Length of the final path
    """
    def __init__(self):
        self.title = "PATH LENGTH"
        self.value = 0