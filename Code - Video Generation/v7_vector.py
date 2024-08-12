import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

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
    ax.set_xlim(0, 60)
    ax.set_ylim(0, 60)
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
    # Check if all robots have reached their targets and stop the animation if true
    if all_reached(initial_positions, final_positions):
        ani.event_source.stop()
        return scat, *circles, *arrows

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

    return scat, *circles, *arrows

def animate_movement(initial_positions, final_positions, radius, file_name):
    fig, ax = setup_plot()
    scat, circles = draw_robot_targets(ax, initial_positions, final_positions, radius)
    arrows = []  # Initialize an empty list to keep track of arrow objects

    def update(frame):
        return update_position_and_visuals(frame, ax, scat, circles, initial_positions, final_positions, radius, arrows)

    global ani
    ani = FuncAnimation(fig, update, frames=np.linspace(0, 1, 200), interval=50, blit=False)
    save_animation(ani, file_name)

def save_animation(ani, file_name):
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    ani.save(file_name, writer=writer)

def main():
    num_nodes, xlim, ylim, radius = 15, (0, 60), (0, 60), 3.0
    min_distance = 2 * radius
    initial_positions, _ = initialize_graph(num_nodes, xlim, ylim, min_distance)
    final_positions, _ = initialize_graph(num_nodes, xlim, ylim, min_distance)
    animate_movement(initial_positions, final_positions, radius, 'v8_retain_robots_target.mp4')

if __name__ == "__main__":
    main()
