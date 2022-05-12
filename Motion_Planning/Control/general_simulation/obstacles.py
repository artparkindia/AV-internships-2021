"""
Obstacle module to simulate obstacles
"""
import json
from PIL import Image
import numpy as np
import cv2

class Obstacles:
    """ 
    Python module to simulate obstacles and their
    collision checking
    """
    def __init__(self, map, instructions = 'instructions.json'):
        self._interpret_instructions(instructions, map)
        self._obstacle_color = [0, 127, 0]
        self._noway_color = [0, 0, 0]

    def check_collision(self, trajectory):
        """ Function to do collision checking """
        return self._check_static_collisions(trajectory)

    def _check_static_collisions(self, trajectory):
        """ Private function to check collision with static obstacles """
        try:
            for (x_i, y_i) in zip(trajectory.x, trajectory.y):
                background_color = self._background_image[int(y_i)][int(x_i)]
                if ((background_color == self._obstacle_color).all() \
                    or (background_color == self._noway_color).all()):
                    return False
            
            return True
        
        except TypeError:
            x_i, y_i = trajectory.x, trajectory.y
            background_color = self._background_image[int(y_i)][int(x_i)]
            if ((background_color == self._obstacle_color).all() \
                or (background_color == self._noway_color).all()):
                return False

            return True

    def _interpret_instructions(self, instructions, map):
        """ Private function to interpret instructions """
        data = None
        with open(instructions) as ins:
            data = json.load(ins)

        kernel = np.ones((5, 5), np.uint8)
        self._background_image = cv2.imread(data[map]["map"])
        self._background_image = cv2.erode(self._background_image, kernel, iterations = data[map]["erosions"])
        # cv2.imshow('dilation', self._background_image)
        # cv2.waitKey(0)