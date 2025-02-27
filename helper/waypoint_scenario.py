import matplotlib.pyplot as plt
import yaml
import numpy as np

#Define scenario fileName
scenario = "Scenario_CPPS_horizontal_robot_2_v3"
fileName = "helper/trajectory/" + scenario + ".yaml"


# Define arena boundaries
arena_x = [-10.1, -10.1, 10.0, 10.0, -10.1]
arena_y = [-4.42, 5.5, 5.5, -4.42, -4.42]

# Define rectangular objects (obstacles & walls)
rectangles = [
    # Standard obstacles (1m x 0.6m)
    {"name": "AS_3_neu", "x": -5.704, "y": -0.1404, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},
    {"name": "AS_5_neu", "x": -3.1071, "y": 2.153, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},
    {"name": "AS_1_neu", "x": 1.529, "y": 2.0651, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},  # Slight diagonal
    {"name": "AS_6_neu", "x": -0.005, "y": -1.553, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},
    {"name": "AS_4_neu", "x": 6.079, "y": 0.3686, "rotate": 1, "angle": 0, "size": (1.0, 0.6)}, # Slight diagonal

    ##original
    # {"name": "AS_3_neu", "x": -5.704, "y": -0.404, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},
    # {"name": "AS_5_neu", "x": -3.183, "y": 1.982, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},
    # {"name": "AS_1_neu", "x": 1.396, "y": 2.106, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},  # Slight diagonal
    # {"name": "AS_6_neu", "x": 2.926, "y": -1.586, "rotate": 0, "angle": 0, "size": (1.0, 0.6)},
    # {"name": "AS_4_neu", "x": 5.437, "y": 0.410, "rotate": 1, "angle": 0, "size": (1.0, 0.6)}, # Slight diagonal
    # New Wall Objects (2.4m x 0.4m)
    #{"name": "Wall_1", "x": -7.0, "y": 2.0, "rotate": 0, "angle": 0, "size": (2.4, 0.4)},
    #{"name": "Wall_2", "x": 4.0, "y": -2.0, "rotate": 1, "angle": 0, "size": (2.4, 0.4)}
]

# Store clicked points
waypoints = []

def rotate_point(x, y, cx, cy, angle):
    """Rotates a point (x, y) around (cx, cy) by 'angle' degrees."""
    angle_rad = np.radians(angle)
    cos_theta, sin_theta = np.cos(angle_rad), np.sin(angle_rad)

    x_new = cos_theta * (x - cx) - sin_theta * (y - cy) + cx
    y_new = sin_theta * (x - cx) + cos_theta * (y - cy) + cy
    return x_new, y_new

def plot_rectangles(ax):
    """Plots obstacles and walls with optional rotation."""
    for rect in rectangles:
        x, y, rotate_flag, angle, size = rect["x"], rect["y"], rect["rotate"], rect["angle"], rect["size"]
        length, width = size

        # Swap length/width for 90-degree rotations
        if rotate_flag == 1:
            length, width = width, length

        # Default rectangle points (centered at x, y)
        half_length, half_width = length / 2, width / 2
        corners = [
            (x - half_length, y - half_width),
            (x + half_length, y - half_width),
            (x + half_length, y + half_width),
            (x - half_length, y + half_width)
        ]

        # Apply arbitrary rotation
        if angle != 0:
            corners = [rotate_point(px, py, x, y, angle) for px, py in corners]

        # Extract X, Y coordinates for plotting
        rect_x, rect_y = zip(*corners)
        rect_x, rect_y = list(rect_x) + [rect_x[0]], list(rect_y) + [rect_y[0]]  # Close the rectangle

        ax.plot(rect_x, rect_y, color='red', alpha=0.8)
        ax.text(x, y, rect["name"], fontsize=8, ha='center', va='center', color='black', fontweight='bold')

# def onclick(event):
#     """Handles mouse clicks on the plot and stores clicked coordinates."""
#     if event.xdata is not None and event.ydata is not None:
#         x, y = round(float(event.xdata), 3), round(float(event.ydata), 3)
#         waypoints.append({"x": x, "y": y})
#         plt.scatter(x, y, color='blue', marker='o')
#         plt.text(x, y, f"({x}, {y})", fontsize=8, ha='right', va='bottom', color='blue')
#         plt.draw()

# def save_yaml():
#     """Saves the clicked waypoints to a YAML file in the correct format step by step."""
#     if waypoints:
#         formatted_waypoints = [{"x": float(wp["x"]), "y": float(wp["y"])} for wp in waypoints]

#         with open(fileName, "w") as file:
#             file.write("waypoints:\n")  # First write "waypoints:"
#             for wp in formatted_waypoints:
#                 file.write(f"  - {{x: {wp['x']}, y: {wp['y']}}}\n")  # Write each waypoint correctly

#         print("Waypoints saved to trajectory.yaml in the correct step-by-step format!")

def onclick(event):
    """Handles mouse clicks on the plot and stores clicked coordinates in order."""
    if event.xdata is not None and event.ydata is not None:
        x, y = round(float(event.xdata), 3), round(float(event.ydata), 3)
        waypoints.append({"x": x, "y": y})
        print(f"Waypoint {len(waypoints)}: x={x}, y={y}")  # Print order for debugging
        plt.scatter(x, y, color='blue', marker='o')
        plt.text(x, y, f"({x}, {y})", fontsize=8, ha='right', va='bottom', color='blue')
        plt.draw()

def save_yaml():
    """Saves the clicked waypoints to a YAML file in the correct format step by step."""
    if waypoints:
        with open(fileName, "w") as file:
            file.write("waypoints:\n")  # First write "waypoints:"
            for wp in waypoints:
                file.write(f"  - {{x: {wp['x']}, y: {wp['y']}}}\n")  # Write each waypoint correctly

        print(f"Waypoints saved to {fileName} in the correct step-by-step format!")


def on_key(event):
    """Handles keyboard input to finalize trajectory collection."""
    if event.key == "enter":
        print("Trajectory creation finished. Saving to YAML...")
        save_yaml()
        plt.close()

def main():
    """Main function to plot the arena and collect waypoints."""
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(arena_x, arena_y, 'k-', linewidth=2, label="Arena Boundary")

    plot_rectangles(ax)

    ax.set_xlim(-11, 11)
    ax.set_ylim(-5, 6)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_title(scenario)

    # Enable 1-meter grid
    ax.set_xticks(range(-11, 12, 1))
    ax.set_yticks(range(-5, 7, 1))
    ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

    # Move legend outside the plot
    #ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), borderaxespad=0)

    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('key_press_event', on_key)

    plt.show()

if __name__ == "__main__":
    main()
