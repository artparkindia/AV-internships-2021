# Control

The Control module consists of the following steps:

Input (Map, Initial Coordinates) -> Path Planning -> Path Interpolation -> Trajectory Planning -> Output (List of Trajectory Points)

The package has been created with Python-3.8.8 and Pip 21.0.1

## Installation

Create a virtual environment and install the required Python libraries.

```bash
pip install -r requirements.txt
```

## Streamlit application

The streamlit application presents an interface to run different combination of the path planning algorithms on different maps. The following command can be used to start the application:

```bash
streamlit run launch_gui.py
```

The different algorithms for different tasks can be selected by the main dropdown buttons. The parameters of the selected algorithms can be adjusted from the left panel.

### Relevant files

- `general_simulation`: This folder contains files for the custom map based simulation. There are 2 main files in the directory: `obstacles.py` and `simulation.py`.

**Note**: Due to the structure of Streamlit, sometimes when the Path Planning code produces some error, the streamlit application does not report it and keeps on loading. If this happens, it is good to restart the application.

## Python application

There are various different main files, that run different algorithms on different settings and maps. They can be run by using the following command:

```bash
python <main_file_name>.py
```

The different `main_file_name` are:

- `main_general.py` This file is the manual python version of the streamlit application. In this we can adjust the different parameters and select algorithms by making changes in the code.

- `main_simulation_many.py` This file is the Frenet Frame simulation in Highway scenario. The car switches between two modes following and velocity keeping to cover the highway. **The simulation scenario is generated randomly, and hence not very smart. Also, the autonomous car only follows the Frenet Frame algorithm and is not very smart to detect other surroundings. Therefore, a random seed has been set to allow for these variations.**

- `main_simulation_static.py` Frenet Frame highway simulation with static obstacles. **Sometimes the algorithm may not work due to the random nature of obstacles.**

- `main_static.py` This is a basic Frenet Frame algorithm on a polynomial curve.

- `main_static_2.py` This is a baisc MPC algorithm on a polynomial curve.

**Note**: For some files, a new directory by the name of `gif` would be required. Some files save the simulation image to this folder which can then be used to create gifs.

### Relevant Files

- `simulation`: This directory contains the Pygame simulation files. There are 3 files in the directory: `obstacles.py`, `path.py` and `simulation.py`.

- `utils`: This directory contains some utility functions for Matplotlib based simulations. There are 3 files in this directory: `obstacles.py`, `plotting.py` and `polynomials.py`.

## Path Planning

Link to [path_planning](./path_planning) module. The algorithms in Path Planning module are:

- A Star
- A Star (NetworkX Implementation)

## Path Interpolation

Link to [path_interpolation](./path_interpolation) module. The algorithms in Path Interpolation module are:

- Linear Interpolation (`linear_spline.py`)
- Cubic Interpolation (`cubic_polynomial.py`)
- Polynomial Interpolation (`vandermonde.py`)

## Trajectory Planning

Link to [trajectory_planning](./trajectory_planning) module. The algorithms in Trajectory Planning are:

- Frenet Frame Trajectory Optimization
- Model Predictive Control

## Results

- The report is present in `REPORT.md` and it's pdf version `REPORT.pdf`.
- The output of the general map simulations are present in `outputs` folder.
- [Here](https://www.youtube.com/watch?v=kPZqT12bLj4) is a video for the Frenet Frame Trajectory Optimization. 