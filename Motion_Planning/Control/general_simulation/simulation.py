"""
Python module to simulate environment

Structure of instructions.json
{
    'map' : <link-to-map-file>,
    'start': [start-pos-x, start-pos-y],
    'end': [end-pos-x, end-pos-y],
}
"""

import matplotlib.pyplot as plt
import matplotlib.image as mpimage
import glob
import os
import shutil
import imageio
import numpy as np
import json

class Simulation:
    """
    Module for constructing simulation environment
    """
    def __init__(self, map, area=100, instructions = 'instructions.json', image_path = 'img'):
        self._area = area
        self._prepare_image_mechanism(image_path)
        self._interpret_instructions(instructions, map)
        self._initialize_ego_car()

    def update(self, path, trajectory_path, other_paths = None):
        """ Function to update the simulation """
        # Get the current plot
        plt.cla()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == "escape" else None]
        )
        # Clear the plots
        self.axis[0].clear(); self.axis[1].clear()


        # Plot the background
        self.axis[0].imshow(self._background_image)
        self.axis[1].imshow(self._background_image)

        # Plot on background
        self._draw_path(path)
        self._draw_trajectory(trajectory_path)

        plt.savefig(f'{self._image_path}/{self._plot_index}.png')
        plt.pause(0.01)

        if trajectory_path is not None:
            if np.hypot(trajectory_path.x[1] - self._end_pos[0], trajectory_path.y[1] - self._end_pos[1]) <= 50.0:
                return True

        self._plot_index += 1
        return False

    def initialize_plots(self):
        """ Function to initialize plots """
        # Get the subplots
        self.fig, self.axis = plt.subplots(1, 2)

    def save_trajectory(self, gif_path):
        """ Function to prepare the final gif """
        # Build GIF
        files = sorted(glob.glob(f'{self._image_path}/*'), key = len)
        with imageio.get_writer(f'{gif_path}/{self._map_name}.gif', mode='I') as writer:
            for filename in files:
                image = imageio.imread(filename)
                writer.append_data(image)

    def save_path(self, points, plot_path):
        """ Function to generate and save a plot """
        plt.close()
        points = np.array(points)
        plt.imshow(self._background_image)
        plt.plot(points[:, 0], points[:, 1])

        plt.savefig(f'{plot_path}/{self._map_name}.png')
        plt.close()

    def save_interpolation(self, polynomial, interpolation_path):
        plt.close()
        # Generate the polynomial from parameter s
        s = np.arange(0, polynomial.s[-1], 10)

        # Generate the points of polynomial
        x = []; y = []
        for s_i in s:
            x_i, y_i = polynomial.calculate_position(s_i)
            x.append(x_i); y.append(y_i)

        plt.imshow(self._background_image)
        plt.plot(x, y)

        plt.savefig(f'{interpolation_path}/{self._map_name}.png')
        plt.close()

    def save_profiles(self, profiles, profile_path):
        """ Function to generate profile plots """
        plt.close()
        plt.figure(figsize=(20, 100))
        fig, axis = plt.subplots(1, 3)
        fig.tight_layout()

        for index in range(3):
            axis[index].plot(profiles[index].x, profiles[index].y)
            axis[index].title.set_text(profiles[index].title)

        plt.savefig(f'{profile_path}/{self._map_name}.png')
        plt.close()

    def _prepare_image_mechanism(self, image_path):
        """ Private function to prepare the gif mechanism """
        self._image_path = image_path
        self._plot_index = 0

        shutil.rmtree(image_path)
        os.mkdir(image_path)


    def _draw_path(self, polynomial, show=False):
        """ Function to plot a polynomial and it's interpolation """
        # Generate the polynomial from parameter s
        s = np.arange(0, polynomial.s[-1], 10)

        # Generate the points of polynomial
        x = []; y = []
        for s_i in s:
            x_i, y_i = polynomial.calculate_position(s_i)
            x.append(x_i); y.append(y_i)

        # Plot the polynomial
        self.axis[0].plot(x, y)
        self.axis[1].plot(x, y)

        if show:
            plt.show()

    def _draw_all_trajectories(self, trajectories, offset):
        """ Function to draw the other trajectory paths """
        pass

    def _draw_trajectory(self, trajectory):
        """ Function to draw the selected trajectory """
        if trajectory == None:
            return
            
        # Plot trajectory and car at current position
        self.axis[1].set_xlim([trajectory.x[1] - self._area, trajectory.x[1] + self._area])
        self.axis[1].set_ylim([trajectory.y[1] + self._area, trajectory.y[1] - self._area])

        self.axis[0].plot(trajectory.x[1], trajectory.y[1], "vc")
        self.axis[1].plot(trajectory.x[1:], trajectory.y[1:], "-or")
        self.axis[1].plot(trajectory.x[1], trajectory.y[1], "vc")

    def _interpret_instructions(self, instructions, map):
        """ Private function to read the instructions from JSON file """
        data = None
        with open(instructions) as ins:
            data = json.load(ins)

        self._map_name = map
        self._background_image = mpimage.imread(data[map]["map"])
        self._start_pos = [data[map]["start"][0], data[map]["start"][1]]
        self._end_pos = [data[map]["end"][0], data[map]["end"][1]]

    def _initialize_ego_car(self):
        """ Private function to initialize ego car """
        self._car_pos = [self._start_pos[0], self._start_pos[1]]