# conda activate multi_robot_assignment_env
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

# Initialize graph nodes (positions) with guaranteed minimum distance
def initialize_graph(num_nodes, xlim, ylim, min_distance):
    positions = []
    while len(positions) < num_nodes:
        new_position = np.random.rand(2) * [xlim[1] - xlim[0], ylim[1] - ylim[0]] + [xlim[0], ylim[0]]
        if all(np.linalg.norm(new_position - np.array(pos)) >= min_distance for pos in positions):
            positions.append(new_position)
    edges = [(i, (i + 1) % num_nodes) for i in range(num_nodes)]  # Circular graph for simplicity
    return np.array(positions), edges

# Update positions with simple collision avoidance
def update_positions(positions, targets, step_size=0.5):
    radius = 3.0
    min_distance = 2 * radius
    new_positions = positions.copy()
    for i in range(len(positions)):
        direction = targets[i] - positions[i]
        distance = np.linalg.norm(direction)
        if distance > step_size:
            new_positions[i] += (direction / distance) * step_size
        else:
            new_positions[i] = targets[i]
    
    # Collision avoidance: ensure no overlapping positions
    for i in range(len(new_positions)):
        for j in range(i + 1, len(new_positions)):
            overlap_distance = np.linalg.norm(new_positions[i] - new_positions[j])
            if overlap_distance < min_distance:
                move_away = (new_positions[j] - new_positions[i]) / overlap_distance * (min_distance - overlap_distance)
                new_positions[i] -= move_away * 0.5
                new_positions[j] += move_away * 0.5
    return new_positions

# Visualization using Matplotlib animation
def animate_movement(initial_positions, final_positions, file_name='animation.mp4'):
    fig, ax = plt.subplots()
    ax.set_xlim(0, 60)
    ax.set_ylim(0, 60)
    scat = ax.scatter(initial_positions[:, 0], initial_positions[:, 1], color='blue', label='Robots')
    target_scat = ax.scatter(final_positions[:, 0], final_positions[:, 1], color='red', marker='x', label='Targets')
    plt.legend()
    
    # Create circle patches for each robot with a dotted line
    circles = [plt.Circle((pos[0], pos[1]), radius=3, color='blue', fill=False, linestyle='dotted', linewidth=1.5) for pos in initial_positions]
    for circle in circles:
        ax.add_patch(circle)

    plt.legend()

    def update(frame):
        nonlocal initial_positions
        initial_positions = update_positions(initial_positions, final_positions, step_size=0.5)
        scat.set_offsets(initial_positions)

        # Update the positions of the circles
        for circle, pos in zip(circles, initial_positions):
            circle.center = (pos[0], pos[1])
        return scat, *circles,

    ani = FuncAnimation(fig, update, frames=np.linspace(0, 1, 200), interval=50, blit=True)

    # Save animation
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    ani.save(file_name, writer=writer)

    # plt.show()

# Main function to initialize and animate robot movements
def main():
    num_nodes = 15
    xlim = (0, 60)
    ylim = (0, 60)
    radius = 3.0
    min_distance = 2 * radius
    initial_positions, _ = initialize_graph(num_nodes, xlim, ylim, min_distance)
    final_positions, _ = initialize_graph(num_nodes, xlim, ylim, min_distance)
    animate_movement(initial_positions, final_positions, 'v5_robots_movement.mp4')

if __name__ == "__main__":
    main()
