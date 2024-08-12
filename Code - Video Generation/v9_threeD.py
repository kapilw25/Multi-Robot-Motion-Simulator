import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

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

def draw_robot_targets_3d(ax, initial_positions, final_positions):
    scat = ax.scatter(initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2], color='blue', label='Robots')
    ax.scatter(final_positions[:, 0], final_positions[:, 1], final_positions[:, 2], color='red', marker='x', label='Targets')
    ax.legend()
    return scat

def update_position_and_visuals_3d(frame, ax, scat, initial_positions, final_positions, radius):
    initial_positions[:] = update_positions(initial_positions, final_positions, radius, step_size=0.5)
    scat._offsets3d = (initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2])

    if all_reached(initial_positions, final_positions):
        print("All robots have reached their targets.")
        ani.event_source.stop()

    return scat

def animate_movement_3d(initial_positions, final_positions, radius, file_name):
    fig, ax = setup_3d_plot()
    scat = draw_robot_targets_3d(ax, initial_positions, final_positions)

    # def update(frame):
    #     return update_position_and_visuals_3d(frame, ax, scat, initial_positions, final_positions, radius)

    # global ani
    # ani = FuncAnimation(fig, update, frames=np.linspace(0, 1, 300), interval=50, blit=False)
    # ani.save(file_name, writer='ffmpeg', fps=15, extra_args=['-vcodec', 'libx264'])
    
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
    

def main():
    num_nodes, xlim, ylim, zlim, radius = 25, (0, 100), (0, 100), (0, 100), 10.0
    min_distance = 2 * radius
    initial_positions = initialize_positions_3d(num_nodes, xlim, ylim, zlim, min_distance)
    final_positions = initialize_positions_3d(num_nodes, xlim, ylim, zlim, min_distance)
    animate_movement_3d(initial_positions, final_positions, radius, 'v9_threeD.mp4')

if __name__ == "__main__":
    main()
