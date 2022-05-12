"""
Simulation obstacles
"""
import pygame
import numpy as np
import random

class Obstacles:
    """ 
    Pygame obstacles
    """
    def __init__(self):
        """ Initialization function """
        self.side = 25
        self.safety_side = 10
        self.objects = []
        self.draw_objects = []
        self.draw_obstacles = []

    def move(self, center_x, center_y, DT):
        """ Function to update the x y coordinates of obstacles """
        angle_pos = np.arctan2(self.obstacles_y - center_y, self.obstacles_x - center_x)
        vx = -1 * self.obstacles_radii * np.sin(angle_pos) * self.obstacles_omega
        vy = self.obstacles_radii * np.cos(angle_pos) * self.obstacles_omega
        
        self.obstacles_x = self.obstacles_x + DT * vx
        self.obstacles_y = self.obstacles_y + DT * vy
        self.obstacles_yaw = np.arctan2(vy, vx)

        self._generate_obstacle_objects()

    def check_collision(self, trajectory):
        """ Function to check collision """
        # Obstacle Avoidance
        for c_object in self.objects:
            for (x, y) in zip(trajectory.x, trajectory.y):
                side = self.side + self.safety_side
                rectangle = pygame.Rect((x - side / 2, y - side / 2), (side, side))
                collision = c_object.colliderect(rectangle)
            
                if collision:
                    return False
        
        return True

    def _generate_obstacle_objects(self):
        """ Private Function to generate obstacle objects """
        self.objects = []
        self.draw_obstacles = []; index  = 0
        for (x, y, yaw) in zip(self.obstacles_x, self.obstacles_y, self.obstacles_yaw):
            rectangle_obstacle = pygame.Rect((x - self.side / 2, y - self.side / 2),
                                            (self.side, self.side))
            self.objects.append(rectangle_obstacle)

            if len(self.draw_objects) < len(self.obstacles_x):
                draw_object = pygame.Surface((self.side, self.side), pygame.SRCALPHA)
                draw_object.fill((255, 0, 0))
                self.draw_objects.append(draw_object)

            rotated_ego = pygame.transform.rotozoom(self.draw_objects[index], -np.degrees(yaw), 1)
            new_coordinates = rotated_ego.get_rect(center = (x, y))
            index += 1

            self.draw_obstacles.append((rotated_ego, new_coordinates))
            

    def intialize_circular_obstacles_static(self, path):
        """ Function to initialize random obstacles on path """
        # Generate random angles for different radii
        angles = [0, np.pi]
        radii = [
            path.radius + lane * path.width for lane in range(int(-path.lanes/2), int(path.lanes/2) + 1)
        ]

        # Generate X and Y coordinates from the random parameters
        x = []; y = []; yaw = []; omega = []; radius_list = []
        for angle in angles:
            for radius in radii:
                radius_list.append(radius)

                # Pick a random angle
                sample = angle + (np.pi * random.random())
                x.append(path.center_x + radius * np.cos(sample))
                y.append(path.center_y + radius * np.sin(sample))

                dy = np.cos(sample); dx = -np.sin(sample)
                yaw.append(np.arctan2(dy, dx))

                omega.append(0)

        self.obstacles_x = np.array(x)
        self.obstacles_y = np.array(y)
        self.obstacles_yaw = np.array(yaw)
        self.obstacles_omega = np.array(omega)
        self.obstacles_radii = np.array(radius_list)
        
        self._generate_obstacle_objects()

    def intialize_circular_obstacles_dynamic(self, path):
        """ Function to initialize random obstacles on path """
        # Generate random angles for different radii
        radii = [
            path.radius + lane * path.width for lane in range(int(-path.lanes/2), int(path.lanes/2) + 1)
        ]

        # Generate X and Y coordinates from the random parameters
        x = []; y = []; yaw = []; omega = []; radius_list = []
        for radius in radii:
            radius_list.append(radius)

            # Pick a random angle
            sample =  (np.pi / 2) -  (np.pi * random.random() / 4)
            x.append(path.center_x + radius * np.cos(sample))
            y.append(path.center_y + radius * np.sin(sample))

            dy = np.cos(sample); dx = -np.sin(sample)
            yaw.append(np.arctan2(dy, dx))

            omega.append((10 / radius) * random.random())
        
        self.obstacles_x = np.array(x)
        self.obstacles_y = np.array(y)
        self.obstacles_yaw = np.array(yaw)
        self.obstacles_omega = np.array(omega)
        self.obstacles_radii = np.array(radius_list)
        
        self._generate_obstacle_objects()

    def intialize_many_circular_obstacles_dynamic(self, path):
        """ Function to initialize random obstacles on path """
        random.seed(7)

        # Generate random angles for different radii
        angles = [np.pi / 4, np.pi / 2, 3 * np.pi / 4, np.pi, 5 * np.pi / 4, 3 * np.pi / 2]
        radii = [
            path.radius + lane * path.width for lane in range(int(-path.lanes/2), int(path.lanes/2) + 1)
        ]

        # Generate X and Y coordinates from the random parameters
        x = []; y = []; yaw = []; omega = []; radius_list = []
        multiplier = 1
        for angle in angles:
            index = 1
            for radius in radii:
                radius_list.append(radius)

                # Pick a random angle
                sample =  angle +  (np.pi * random.random() / 4)
                x.append(path.center_x + radius * np.cos(sample))
                y.append(path.center_y + radius * np.sin(sample))

                dy = np.cos(sample); dx = -np.sin(sample)
                yaw.append(np.arctan2(dy, dx))

                if index % 3 == 0:
                    omega.append(-1 * (8 / radius) * random.random() / multiplier)
                    multiplier += multiplier * 0.2
                else:
                    omega.append((10 / radius) * random.random() * multiplier)

                index += 1
        
        self.obstacles_x = np.array(x)
        self.obstacles_y = np.array(y)
        self.obstacles_yaw = np.array(yaw)
        self.obstacles_omega = np.array(omega)
        self.obstacles_radii = np.array(radius_list)
        
        self._generate_obstacle_objects()
