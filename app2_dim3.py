# conda activate multi_robot_assignment_env
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import streamlit as st
import os

def generate_random_position_3d(xlim, ylim, zlim):
    return np.random.rand(3) * [xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]] + [xlim[0], ylim[0], zlim[0]]

def is_position_valid(new_position, positions, min_distance):
    return all(np.linalg.norm(new_position - np.array(pos)) >= min_distance for pos in positions)

def initialize_positions_3d(num_nodes, xlim, ylim, zlim, min_distance):
    positions = []
    while len(positions) < num_nodes:
        new_position = generate_random_position_3d(xlim, ylim, zlim)
        if is_position_valid(new_position, positions, min_distance):
            positions.append(new_position)
    return np.array(positions)

def move_towards_target(position, target, step_size):
    direction = target - position
    distance = np.linalg.norm(direction)
    if distance == 0:
        return position
    return position + (direction / distance) * min(step_size, distance)

def update_positions(positions, targets, radius, step_size=0.5):
    min_distance = 2 * radius
    new_positions = [move_towards_target(pos, tar, step_size) for pos, tar in zip(positions, targets)]
    return new_positions

def all_reached(positions, targets, threshold=0.5):
    return all(np.linalg.norm(pos - tar) <= threshold for pos, tar in zip(positions, targets))

def setup_3d_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_zlim(0, 100)
    return fig, ax

def draw_robot_targets_3d(ax, initial_positions, final_positions, radius):
    # Adjust the 's' parameter to scale with 'radius^2' for visibility in 3D
    robot_size = np.pi * (radius ** 2) * 10  # Adjust multiplication factor as needed for visibility
    scat = ax.scatter(initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2], 
                      s=robot_size, color='blue', label='Robots', depthshade=True)
    target_size = np.pi * (2 ** 2) * 10  # Smaller size for targets
    ax.scatter(final_positions[:, 0], final_positions[:, 1], final_positions[:, 2], 
               s=target_size, color='red', marker='x', label='Targets', depthshade=True)
    ax.legend()
    return scat

def update_position_and_visuals_3d(frame, ax, scat, initial_positions, final_positions, radius, ani):
    # Update positions
    initial_positions[:] = update_positions(initial_positions, final_positions, radius, step_size=0.5)
    scat._offsets3d = (initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2])
    
    # Update the size ensuring it is always treated as a flat array
    size_array = np.full((len(initial_positions),), np.pi * (radius ** 2) * 10)
    scat.set_sizes(size_array)

    if all_reached(initial_positions, final_positions):
        print("All robots have reached their targets.")
        ani.event_source.stop()

    return scat


def main():
    st.title("3D Multi-Robot Task Simulator")
    num_nodes = st.sidebar.slider("Number of Nodes", 10, 30, 15)
    xlim_min, xlim_max = st.sidebar.slider("X-axis Limits", 0, 100, (10, 90))
    ylim_min, ylim_max = st.sidebar.slider("Y-axis Limits", 0, 100, (10, 90))
    zlim_min, zlim_max = st.sidebar.slider("Z-axis Limits", 0, 100, (10, 90))
    radius = st.sidebar.slider("Radius", 1, 10, 3)
    min_distance = 2 * radius
    video_file = 'robot_simulation_3d.mp4'

    if st.button('Run Simulation'):
        initial_positions = initialize_positions_3d(num_nodes, (xlim_min, xlim_max), (ylim_min, ylim_max), (zlim_min, zlim_max), min_distance)
        final_positions = initialize_positions_3d(num_nodes, (xlim_min, xlim_max), (ylim_min, ylim_max), (zlim_min, zlim_max), min_distance)
        
        # Generate and save the animation
        fig, ax = setup_3d_plot()
        scat = draw_robot_targets_3d(ax, initial_positions, final_positions, radius)

        def update(frame):
            update_position_and_visuals_3d(frame, ax, scat, initial_positions, final_positions, radius, ani)
            angle = frame % 360
            ax.view_init(30, angle)
            return scat

        ani = FuncAnimation(fig, update, frames=np.arange(0, 360, 1), interval=50, blit=False)
        ani.save(video_file, writer='ffmpeg', fps=15, extra_args=['-vcodec', 'libx264'])

        # Display the video in the Streamlit app
        st.video(video_file)

        # Optional: Clean up by removing the video file if you don't want it to accumulate on the server
        os.remove(video_file)

if __name__ == "__main__":
    st.set_option('deprecation.showPyplotGlobalUse', False)
    main()