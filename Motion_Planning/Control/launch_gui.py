"""
GUI Application using Streamlit
"""
from general_simulation.simulation import Simulation
from general_simulation.obstacles import Obstacles
from path_planning.selector import *
from path_interpolation.selector import *
from trajectory_planning.selector import *

import streamlit as st
import base64
import matplotlib.image as mpimage
import json


# Read the instructions file
def _interpret_instructions(instructions = 'instructions.json'):
    """ Function to interpret the json instructions """
    data = None
    with open(instructions) as ins:
        data = json.load(ins)

    return data

# To read local gif file
def _show_gif(gif_path):
    file_ = open(gif_path, "rb")
    contents = file_.read()
    data_url = base64.b64encode(contents).decode('utf-8')
    file_.close()

    st.markdown(
        f'<img src="data:image/gif;base64,{data_url}">',
        unsafe_allow_html = True,
    )

# Show an image using markdown
def _show_image(img_path):
    file_ = open(img_path, "rb")
    contents = file_.read()
    data_url = base64.b64encode(contents).decode('utf-8')
    file_.close()

    st.markdown(
        f"<center><img src='data:image;base64,{data_url}' style='width: 350px;'></center>",
        unsafe_allow_html = True    
    )

instructions = _interpret_instructions()

# Title
st.title('MOTION PLANNING AND CONTROL')

# Map Selection
selected_map = st.selectbox('Select a Map', list(instructions.keys()))
_show_image(instructions[selected_map]["map"])
simulation = Simulation(selected_map)
obstacles = Obstacles(selected_map)

# Path Planning
st.title('Path Planning')

# Selection variables
selected_path_planner = st.selectbox("Select Algorithm", path_planning_algorithms)
path_planner = select_path_planner(selected_path_planner)

st.sidebar.title("PATH PLANNING PARAMETERS")
for parameter in path_planner.parameters:
    if parameter.type == 'selectbox':
        parameter.set_value(
            st.sidebar.selectbox(parameter.title, list(parameter.selections.keys()))
        )
    elif parameter.type == 'slider':
        parameter.set_value(
            st.sidebar.slider(parameter.title, parameter.min_value, parameter.max_value, parameter.value, parameter.step)
        )

# Planning the path
start_pos, end_pos = instructions[selected_map]["start"], instructions[selected_map]["end"]
points = path_planner.plan_path(obstacles, start_pos, end_pos)
simulation.save_path(points, 'outputs/path_planning')

st.markdown("## Path Planning Statistics")
for statistic in path_planner.statistics:
    st.text(f"{statistic.title:<30}:{statistic.value:>15}")

map_image = mpimage.imread(f'outputs/path_planning/{selected_map}.png')
st.image(map_image)

# Path Interpolation
st.title('Path Interpolation')

# Selection variables
selected_path_interpolator = st.selectbox("Select Method", path_interpolation_methods)
path_interpolator = select_path_interpolator(selected_path_interpolator)

st.sidebar.title("PATH INTERPOLATION PARAMETERS")
for parameter in path_interpolator.parameters:
    if parameter.type == 'selectbox':
        parameter.set_value(
            st.sidebar.selectbox(parameter.title, list(parameter.selections.keys()))
        )
    elif parameter.type == 'slider':
        parameter.set_value(
            st.sidebar.slider(parameter.title, parameter.min_value, parameter.max_value, parameter.value, parameter.step)
        )

# Interpolate the path
path = path_interpolator.interpolate(points)
simulation.save_interpolation(path, 'outputs/path_interpolation')

map_image = mpimage.imread(f'outputs/path_interpolation/{selected_map}.png')
st.image(map_image)

# Trajectory Planning
st.title('Trajectory Planning')

# Selected variables
selected_trajectory_planner = st.selectbox("Select Algorithm", trajectory_planning_algorithms)
trajectory_planner = select_trajectory_planner(selected_trajectory_planner)

st.sidebar.title("TRAJECTORY PLANNING PARAMETERS")
for parameter in trajectory_planner.parameters:
    if parameter.type == 'selectbox':
        parameter.set_value(
            st.sidebar.selectbox(parameter.title, list(parameter.selections.keys()))
        )
    elif parameter.type == 'slider':
        parameter.set_value(
            st.sidebar.slider(parameter.title, parameter.min_value, parameter.max_value, parameter.value, parameter.step)
        )

# Trajectory Planning
simulation.initialize_plots()

start_trajectory = st.button("Start Trajectory Planning")
loading_image_placeholder = st.empty()
loop = 0

if start_trajectory:
    while start_trajectory:
        try:
            trajectory_path, algorithm_path = trajectory_planner.plan_trajectory(path, obstacles)
            completed = simulation.update(path, trajectory_path, None)

            if completed:
                simulation.save_trajectory(f'outputs/trajectory_planning')
                simulation.save_profiles(trajectory_planner.profiles, 'outputs/profiles')
                break

            trajectory_planner.set_initial_conditions_path(algorithm_path, 1)

            loading_image = mpimage.imread(f'img/{loop}.png')
            loading_image_placeholder.image(loading_image)
        
        except:
            simulation.save_trajectory(f'outputs/trajectory_planning')
            simulation.save_profiles(trajectory_planner.profiles, 'outputs/profiles')

        loop += 1

loading_image_placeholder.empty()
try:
    _show_gif(f'outputs/trajectory_planning/{selected_map}.gif')
except FileNotFoundError:
    pass

try:
    # Plot Profiles
    map_image = mpimage.imread(f'outputs/profiles/{selected_map}.png')
    st.image(map_image)
except FileNotFoundError:
    pass