import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Configuration parameters
time_steps = 100  # Number of time steps for the simulation
initial_position = np.array([0, 0])  # Starting position (x, y)
average_error = 0.5  # Average drift error per time step (units)
max_error = 1.0  # Maximum drift error per time step (units)
heatmap_size = (512, 512)  # Size of the heatmap grid

def generate_heatmap(size, num_spots=10, min_spot_size = 30, max_spot_size=100, max_amplitude=2*np.pi, min_amplitude=0):
    # Create a random array to store the heatmap values
    heatmap = np.zeros(size)

    # Generate random heat and cold spots
    for _ in range(num_spots):
        # Randomly select spot characteristics
        spot_center = (np.random.randint(0, size[0]), np.random.randint(0, size[1]))
        spot_size = np.random.randint(min_spot_size, max_spot_size)
        spot_amplitude = np.random.uniform(min_amplitude, max_amplitude)

        # Randomly stretch the Gaussian in different directions (elliptical distortion)
        stretch_x = np.random.uniform(1, 5)  # Stretch factor in X direction
        stretch_y = np.random.uniform(1, 5)  # Stretch factor in Y direction

        # Generate the distorted spot using a Gaussian distribution
        Y, X = np.ogrid[:size[0], :size[1]]
        distance = ((X - spot_center[1]) / stretch_x)**2 + ((Y - spot_center[0]) / stretch_y)**2
        spot = spot_amplitude * np.exp(-distance / (2 * (spot_size**2)))

        # Add the spot to the heatmap (making sure to clip the values to avoid overflow)
        heatmap += np.clip(spot, min_amplitude, max_amplitude)

    # Apply a lower amount of smoothing to preserve more peaks/valleys
    # heatmap = gaussian_filter(heatmap, sigma=1)  # lower sigma value

    # Normalize the heatmap to have values between 0 and 1
    # heatmap = (heatmap - np.min(heatmap)) / (np.max(heatmap) - np.min(heatmap))
    return np.mod(heatmap, 2*np.pi)



# Drift step with directional heatmap input
def drift_step_with_heatmap(current_position, directions, grid_size):
    x, y = current_position
    grid_x, grid_y = int(x % grid_size[0]), int(y % grid_size[1])
    drift_angle = directions[grid_x, grid_y]
    drift_magnitude = np.random.uniform(average_error, max_error)

    drift_vector = drift_magnitude * np.array([np.cos(drift_angle), np.sin(drift_angle)])
    next_position = current_position + drift_vector

    return next_position

# # # Initialize variables
directions = generate_heatmap(heatmap_size)
# # Save the directions heatmap to a file
with open('/home/hugo/Documents/Thesis_ARGoS/position_direction_offset.txt', 'w') as f:
    # f.write('{')
    for row in directions:
        f.write('{')
        f.write(', '.join(map(str, row)))
        f.write('},\n')
    # f.write('}\n')
# positions = [initial_position]

# # Read the directions heatmap from a file
# directions = []
# with open('/home/hugo/Documents/Thesis_ARGoS/position_direction_offset.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         if line.strip().startswith('{') and line.strip().endswith('},'):
#             row = line.strip()[1:-2].split(', ')
#             directions.append([float(val) for val in row])
# directions = np.array(directions)

# Visualize the directional heatmap using colors to represent angles
angle_colors = (directions)  # Normalize angles to [0, 1] for coloring
plt.figure(figsize=(8, 8))
plt.imshow(angle_colors, cmap='twilight', origin='lower', extent=[0, heatmap_size[0], 0, heatmap_size[1]])
plt.colorbar(label='Angle (normalized)')
plt.title('Directional Heatmap (Color-encoded)')
plt.xlabel('X Direction')
plt.ylabel('Y Direction')
plt.grid(False)

# Add arrows representing directions
# x_indices, y_indices = np.meshgrid(np.arange(heatmap_size[0]), np.arange(heatmap_size[1]))
# arrows_x = np.cos(directions)
# arrows_y = np.sin(directions)
# plt.quiver(x_indices, y_indices, arrows_x, arrows_y, scale=30, color='black', alpha=0.6)
plt.show()

# Fit a smooth equation to approximate the heatmap
x_coords, y_coords = np.meshgrid(np.linspace(0, heatmap_size[0] - 1, heatmap_size[0]), 
                                 np.linspace(0, heatmap_size[1] - 1, heatmap_size[1]))
x_flat, y_flat = x_coords.ravel(), y_coords.ravel()
angle_flat = directions.ravel()

# Define a fitting function
# More complex form with additional terms to improve fitting
def fit_function(coords, a, b, c, d, e, f, g, h):
    x, y = coords
    return (a * np.sin(b * x + c * y) +
            d * np.cos(e * x + f * y) +
            g * np.sin(h * (x + y)))

# Fit the model
initial_guess = [1, 0.1, 0.1, 1, 0.1, 0.1, 0.5, 0.1]
params, _ = curve_fit(fit_function, (x_flat, y_flat), angle_flat, p0=initial_guess)

# Display the fitted function
print("Approximated equation for directional heatmap angles:")
print(f"f(x, y) = {params[0]:.3f} * sin({params[1]:.3f} * x + {params[2]:.3f} * y) + "
      f"{params[3]:.3f} * cos({params[4]:.3f} * x + {params[5]:.3f} * y) + "
      f"{params[6]:.3f} * sin({params[7]:.3f} * (x + y))")

# # Simulate drift over time using heatmap for bias
# for _ in range(time_steps):
#     current_position = positions[-1]
#     next_position = drift_step_with_heatmap(current_position, directions, heatmap_size)
#     positions.append(next_position)

# # Convert position history to numpy array for plotting
# positions = np.array(positions)

# # Plot the drift path
# plt.figure(figsize=(10, 6))
# plt.plot(positions[:, 0], positions[:, 1], marker='o', linestyle='-', markersize=4)
# plt.scatter(positions[0, 0], positions[0, 1], color='green', label='Start')
# plt.scatter(positions[-1, 0], positions[-1, 1], color='red', label='End')
# plt.title('Simulated Position Drift Over Time with Heatmap Bias')
# plt.xlabel('X Position')
# plt.ylabel('Y Position')
# plt.grid(True)
# plt.legend()
# plt.axis('equal')
# plt.show()
