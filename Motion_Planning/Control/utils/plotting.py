"""
Module for plotting
"""
import numpy as np
import matplotlib.pyplot as plt

def plot_interpolation(polynomial, show=False):
    """ Function to plot a polynomial and it's interpolation """
    # Generate the polynomial from parameter s
    s = np.arange(0, polynomial.s[-1], 0.1)

    # Generate the points of polynomial
    x = []; y = []
    for s_i in s:
        x_i, y_i = polynomial.calculate_position(s_i)
        x.append(x_i); y.append(y_i)

    # Plot the polynomial
    plt.plot(x, y)

    if show:
        plt.show()

def plot_trajectory(trajectory):
    """ Function to plot a a trajectory """
    plt.scatter(trajectory.x, trajectory.y)
    plt.show()

def plot_obstacles(obstacles):
    """ Function to plot obstacles """
    if obstacles is not None:
        plt.plot(obstacles.x, obstacles.y, 'xk')

def plot_simulation(path, trajectory_path, obstacles, index):
    """  Function to plot a scene of the simulation """
    plt.cla()
    # plt.gcf().canvas.mpl_connect(
    #     'key_release_event',
    #     lambda event: [exit(0) if event.key == "escape" else None]
    # )
    
    plot_interpolation(path)
    plot_obstacles(obstacles)

    plt.plot(trajectory_path.x[1:], trajectory_path.y[1:], "-or")
    plt.plot(trajectory_path.x[1], trajectory_path.y[1], "vc")
    plt.savefig(f'gif/{index}.png')
    plt.pause(0.01)