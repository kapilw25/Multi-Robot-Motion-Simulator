# conda activate multi_robot_assignment_env
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import streamlit as st
import matplotlib.animation as animation
import os

####################################################################################################################################
# 2 TWO_DIMENSIONS CODE

def generate_random_position(xlim, ylim):
    return np.random.rand(2) * [xlim[1] - xlim[0], ylim[1] - ylim[0]] + [xlim[0], ylim[0]]

def is_position_valid(new_position, positions, min_distance):
    return all(np.linalg.norm(new_position - np.array(pos)) >= min_distance for pos in positions)

def initialize_positions(num_nodes, xlim, ylim, min_distance):
    positions = []
    while len(positions) < num_nodes:
        new_position = generate_random_position(xlim, ylim)
        if is_position_valid(new_position, positions, min_distance):
            positions.append(new_position)
    return positions

def create_edges(num_nodes):
    return [(i, (i + 1) % num_nodes) for i in range(num_nodes)]

def initialize_graph(num_nodes, xlim, ylim, min_distance):
    positions = initialize_positions(num_nodes, xlim, ylim, min_distance)
    edges = create_edges(num_nodes)
    return np.array(positions), edges

def move_towards_target(position, target, step_size):
    direction = target - position
    distance = np.linalg.norm(direction)
    if distance == 0:
        return position  # Return the current position if it is already at the target
    return position + (direction / distance) * min(step_size, distance)


def apply_collision_avoidance(position, positions, min_distance):
    for other_position in positions:
        distance = np.linalg.norm(other_position - position)
        if distance < min_distance and distance > 0:
            move_away = (other_position - position) / distance * min_distance
            position -= move_away * 0.5
            other_position += move_away * 0.5
    return position

def update_positions(positions, targets, radius, step_size=0.5):
    min_distance = 2 * radius
    new_positions = [move_towards_target(pos, tar, step_size) for pos, tar in zip(positions, targets)]
    for i, position in enumerate(new_positions):
        new_positions[i] = apply_collision_avoidance(position, new_positions, min_distance)
    return new_positions

def all_reached(positions, targets, threshold=0.5):
    return all(np.linalg.norm(pos - tar) <= threshold for pos, tar in zip(positions, targets))

def setup_plot():
    fig, ax = plt.subplots()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    return fig, ax

def draw_robot_targets(ax, initial_positions, final_positions, radius):
    scat = ax.scatter(initial_positions[:, 0], initial_positions[:, 1], color='blue', label='Robots')
    ax.scatter(final_positions[:, 0], final_positions[:, 1], color='red', marker='x', label='Targets')
    plt.legend()
    circles = [plt.Circle((pos[0], pos[1]), radius, color='blue', fill=False, linestyle='dotted', linewidth=1.5) for pos in initial_positions]
    for circle in circles:
        ax.add_patch(circle)
    return scat, circles

def update_position_and_visuals(frame, ax, scat, circles, initial_positions, final_positions, radius, arrows):
    # Update positions
    initial_positions[:] = update_positions(initial_positions, final_positions, radius, step_size=0.5)
    scat.set_offsets(initial_positions)

    # Manage arrows: Remove old, add new
    for arrow in arrows:
        arrow.remove()
    arrows.clear()
    arrows.extend([
        ax.arrow(pos[0], pos[1], 2 * radius * (tar[0] - pos[0]) / np.linalg.norm(tar - pos),
                 2 * radius * (tar[1] - pos[1]) / np.linalg.norm(tar - pos), head_width=1, head_length=1, fc='green', ec='green')
        for pos, tar in zip(initial_positions, final_positions)
    ])

    # Update the positions of the circles
    for circle, pos in zip(circles, initial_positions):
        circle.center = (pos[0], pos[1])

    # Check if all robots have reached their targets and optionally print a message or take some other action
    if all_reached(initial_positions, final_positions):
        print("All robots have reached their targets.")  # Informative message instead of stopping the animation

    return scat, *circles, *arrows



def animate_movement(initial_positions, final_positions, radius, file_name):
    fig, ax = setup_plot()
    scat, circles = draw_robot_targets(ax, initial_positions, final_positions, radius)
    arrows = []  # Initialize an empty list to keep track of arrow objects

    def update(frame):
        return update_position_and_visuals(frame, ax, scat, circles, initial_positions, final_positions, radius, arrows)

    global ani
    ani = FuncAnimation(fig, update, frames=np.linspace(0, 1, 300), interval=50, blit=False) # frames = 300 for 20 seconds video for num_nodes = 25
    save_animation(ani, file_name)

def save_animation(ani, file_name):
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    ani.save(file_name, writer=writer)

####################################################################################################################################
# 3 THREE_DIMENSIONS CODE
def generate_random_position_3d(xlim, ylim, zlim):
    return np.random.rand(3) * [xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]] + [xlim[0], ylim[0], zlim[0]]

def is_position_valid_3d(new_position, positions, min_distance):
    return all(np.linalg.norm(new_position - np.array(pos)) >= min_distance for pos in positions)

def initialize_positions_3d(num_nodes, xlim, ylim, zlim, min_distance):
    positions = []
    while len(positions) < num_nodes:
        new_position = generate_random_position_3d(xlim, ylim, zlim)
        if is_position_valid_3d(new_position, positions, min_distance):
            positions.append(new_position)
    return np.array(positions)

def move_towards_target_3d(position, target, step_size):
    direction = target - position
    distance = np.linalg.norm(direction)
    if distance == 0:
        return position
    return position + (direction / distance) * min(step_size, distance)

def update_positions_3d(positions, targets, radius, step_size=0.5):
    min_distance = 2 * radius
    new_positions = [move_towards_target_3d(pos, tar, step_size) for pos, tar in zip(positions, targets)]
    return new_positions

def all_reached_3d(positions, targets, threshold=0.5):
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

def update_position_and_visuals_3d(frame, ax, scat, initial_positions, final_positions, radius):
    # Update positions
    initial_positions[:] = update_positions_3d(initial_positions, final_positions, radius, step_size=0.5)
    scat._offsets3d = (initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2])
    
    # Update the size ensuring it is always treated as a flat array
    size_array = np.full((len(initial_positions),), np.pi * (radius ** 2) * 10)
    scat.set_sizes(size_array)

    if all_reached_3d(initial_positions, final_positions):
        print("All robots have reached their targets.")
        ani.event_source.stop()

    return scat

def animate_movement_3d(initial_positions, final_positions, radius, file_name):
    fig, ax = setup_3d_plot()
    scat = draw_robot_targets_3d(ax, initial_positions, final_positions, radius)

    def update(frame):
        # Update positions and visuals
        update_position_and_visuals_3d(frame, ax, scat, initial_positions, final_positions, radius)
        
        # Rotate the view
        angle = frame % 360
        ax.view_init(30, angle)  # 30 is the elevation angle, you can adjust it as needed
        
        return scat

    global ani
    # Use more frames to achieve a smoother rotation
    ani = FuncAnimation(fig, update, frames=np.arange(0, 360, 1), interval=50, blit=False)
    ani.save(file_name, writer='ffmpeg', fps=15, extra_args=['-vcodec', 'libx264'])


####################################################################################################################################
# MAIN FUNCTION  with Streamlit integration

def main():
    st.title("Multi-Robot Motion Simulator")
    num_nodes = st.sidebar.slider("Number of Nodes", 10, 50, 25)
    xlim_min, xlim_max = st.sidebar.slider("X-axis Limits", 0, 100, (0, 90))
    ylim_min, ylim_max = st.sidebar.slider("Y-axis Limits", 0, 100, (0, 90))
    zlim_min, zlim_max = st.sidebar.slider("Z-axis Limits", 0, 100, (0, 90))
    radius = st.sidebar.slider("Radius", 1, 10, 3)
    min_distance = 2 * radius
    video_file_2d = 'robot_simulation_2d.mp4'
    video_file_3d = 'robot_simulation_3d.mp4'

    if st.button('Run 2D Simulation'):
        initial_positions, _ = initialize_graph(num_nodes, (xlim_min, xlim_max), (ylim_min, ylim_max), min_distance)
        final_positions, _ = initialize_graph(num_nodes, (xlim_min, xlim_max), (ylim_min, ylim_max), min_distance)
        animate_movement(initial_positions, final_positions, radius, video_file_2d)
        st.video(video_file_2d)
        os.remove(video_file_2d)
        
    if st.button('Run 3D Simulation'):
        initial_positions = initialize_positions_3d(num_nodes, (xlim_min, xlim_max), (ylim_min, ylim_max), (zlim_min, zlim_max), min_distance)
        final_positions = initialize_positions_3d(num_nodes, (xlim_min, xlim_max), (ylim_min, ylim_max), (zlim_min, zlim_max), min_distance)
        animate_movement_3d(initial_positions, final_positions, radius, video_file_3d)
        st.video(video_file_3d)
        os.remove(video_file_3d)

if __name__ == "__main__":
    st.set_option('deprecation.showPyplotGlobalUse', False)
    main()