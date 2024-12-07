import matplotlib.pyplot as plt
import matplotlib.patches as patches

def generate_objects_from_grid(grid, resolution, width, height):
    objects = []
    for i in range(height):
        for j in range(width):
            if grid[i][j] == 100:  # If there's an object (value 100)
                # Create a 3x3 box around the object
                x = j * resolution - resolution  # Left side of the box
                y = i * resolution - resolution  # Top side of the box
                objects.append([x, y, 1 * resolution, 1 * resolution])
    return objects

# Example grid data (replace this with your actual grid)
grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 100, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 100, 0, 0, 0, 0, 0, 100, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

width = 10  # Number of columns
height = 10  # Number of rows
resolution = 10  # Resolution in pixels (each grid cell corresponds to this size)

# Generate objects from the grid
objects = generate_objects_from_grid(grid, resolution, width, height)

# Adjusted bounding boxes (f_area) with agent radius
agent_radius = 10
f_area = [[obj[0] - agent_radius, obj[1] - agent_radius, obj[2] + 2 * agent_radius, obj[3] + 2 * agent_radius] for obj in objects]

# Create plot
fig, ax = plt.subplots()

# Add the adjusted bounding boxes as rectangles
for area in f_area:
    rect = patches.Rectangle((area[0], area[1]), area[2], area[3], linewidth=1, edgecolor='r', facecolor='none')
    ax.add_patch(rect)

# Optionally, plot path (example path, replace with your actual path)
path = [[50, 50], [100, 100], [150, 200], [200, 250], [300, 300], [500, 400]]
x_values = [point[0] for point in path]
y_values = [point[1] for point in path]
ax.plot(x_values, y_values, color='b', marker='o', label='Path', linewidth=1)

# Display plot
ax.set_xlabel('X values')
ax.set_ylabel('Y values')
ax.set_title('Plot of Objects and Path')
ax.grid(True)
plt.show()
