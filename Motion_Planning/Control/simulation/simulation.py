"""
Python module to simulate pygame environment

Reference: https://www.youtube.com/watch?v=Cj6tAQe7UCY
"""
import pygame
import numpy as np

class Simulation:
    """ 
    Module for constructing a pygame simulation
    """
    def __init__(self, width=480, height=360, side=25):
        """ Initialization function """
        self.side = side
        self.obstacles = []

        self.rectangle_ego = None
        self.camera = pygame.Vector2(width/2, height/2)
        self.center_pos = pygame.Vector2(width/2, height/2)

        pygame.init()
        pygame.display.set_caption("Frenet Frame Trajectory Optimization")

        self.screen = pygame.display.set_mode((width, height))
        self.screen.fill((255, 255, 255))
        pygame.display.update()

    def update(self, path, trajectory_path, obstacles, other_paths = None):
        """ Function to update the simulation frame """
        # Erase the screen
        self.screen.fill((255, 255, 255))

        car_pos = pygame.Vector2(trajectory_path.x[0], trajectory_path.y[0])
        heading = car_pos - self.camera
        self.camera = self.camera + heading
        self.offset = self.center_pos - self.camera

        # Draw everything again
        self._draw_circle_path(path)
        self._draw_rectangle_obstacles(obstacles)
        if other_paths is not None:
            self._draw_all_trajectories(other_paths)
        if trajectory_path is not None:
            self._draw_trajectory(trajectory_path)

        pygame.display.update()

    def _draw_circle_path(self, path):
        """ Private function to draw a path """
        # Center blue line
        offset_x = self.offset[0]; offset_y = self.offset[1]
        pygame.draw.circle(self.screen, (0, 0, 255), (path.center_x + offset_x, path.center_y + offset_y), path.radius, width=1)

        # Lane seperations
        for lane in range(-path.lanes, path.lanes + 1, 2):
            radius = path.radius + lane * path.width / 2
            if lane == -path.lanes or lane == path.lanes:
                pygame.draw.circle(self.screen, (0, 0, 0), (path.center_x + offset_x, path.center_y + offset_y), radius, width=1)
            else:
                pygame.draw.circle(self.screen, (0, 0, 0, 0.5), (path.center_x + offset_x, path.center_y + offset_y), radius, width=1)

    def _draw_rectangle_obstacles(self, obstacles):
        """ Private function to draw obstacles as rectangles """
        if obstacles is not None:
            for (obstacle_rot, obstacle_trs) in obstacles.draw_obstacles:
                self.screen.blit(obstacle_rot, obstacle_trs.topleft + self.offset)


    def _draw_trajectory(self, trajectory):
        """ Private function to draw the trajectory and the object """
        x = trajectory.x[1]; y = trajectory.y[1]; yaw = trajectory.yaw[1]
        line_coordinates = [pygame.Vector2(x_i, y_i) + self.offset for (x_i, y_i) in zip(trajectory.x[1:], trajectory.y[1:])]
        
        if self.rectangle_ego is None:
            self.rectangle_ego = pygame.Surface((self.side, self.side), pygame.SRCALPHA)
            self.rectangle_ego.fill((150, 150, 0))

        rotated_ego = pygame.transform.rotozoom(self.rectangle_ego, -np.degrees(yaw), 1)
        new_coordinates = rotated_ego.get_rect(center = (x, y))

        self.screen.blit(rotated_ego, new_coordinates.topleft + self.offset)
        pygame.draw.lines(self.screen, (0, 255, 0), False, line_coordinates)

    def _draw_all_trajectories(self, trajectories):
        """ Private function to draw the trajectory and the object """
        for trajectory in trajectories:
            line_coordinates = [pygame.Vector2(x_i, y_i) + self.offset for (x_i, y_i) in zip(trajectory.x[1:], trajectory.y[1:])]
            pygame.draw.lines(self.screen, (150, 150, 0), False, line_coordinates)