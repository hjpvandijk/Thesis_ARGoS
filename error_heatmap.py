import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import scipy.stats as stats
import scipy.optimize as opt
import math

min_error = 0.004
max_error = 1.033
mean_error = 0.227
median_error = 0.193
std_dev = 0.154

def gamma_params(mean, median, std_dev):
    """Estimate shape (k) and scale (θ) parameters for the Gamma distribution."""
    k_init = (mean / std_dev) ** 2  # Initial estimate for shape
    
    def median_error(k):
        """Objective function to minimize median error."""
        theta = std_dev**2 / mean  # Scale parameter from mean and std_dev
        median_est = stats.gamma.median(k, scale=theta)  # Compute actual median
        return (median_est - median) ** 2
    
    # Optimize k to match median
    result = opt.minimize_scalar(median_error, bounds=(0.1, 100), method='bounded')
    k_opt = result.x
    theta_opt = std_dev**2 / mean  # Recalculate scale with optimized k
    
    return k_opt, theta_opt

def truncated_gamma(k, theta, min_val, max_val, size):
    """Generate samples from a truncated Gamma distribution within [min_val, max_val]."""
    total_samples = size[0] * size[1]  # Flattened number of required samples
    samples = []
    
    while len(samples) < total_samples:
        new_samples = stats.gamma.rvs(k, scale=theta, size=total_samples - len(samples))
        new_samples = new_samples[(new_samples >= min_val) & (new_samples <= max_val)]
        samples.extend(new_samples)  # Append valid samples
    
    samples = np.array(samples[:total_samples])  # Trim to required number
    return samples.reshape(size)  # Reshape into (width, height) matrix

# def check_gamma_distribution(k, theta, min_val, max_val):
#     percentiles = stats.gamma.ppf([0.01, 0.99], k, scale=theta)  # 1% and 99% percentiles
#     return percentiles
# gamma_shape = (mean_error ** 2) / (std_dev ** 2)
# gamma_scale = (std_dev ** 2) / mean_error

gamma_shape, gamma_scale = gamma_params(mean_error, median_error, std_dev)
# estimated_min, estimated_max = check_gamma_distribution(gamma_shape, gamma_scale, min_error, max_error)
# print(f"Estimated Shape (k): {gamma_shape:.4f}")
# print(f"Estimated Scale (θ): {gamma_scale:.4f}")
# print(f"Estimated 1% percentile: {estimated_min:.4f}")
# print(f"Estimated 99% percentile: {estimated_max:.4f}")
# exit()

# Configuration parameters
time_steps = 100  # Number of time steps for the simulation
initial_position = np.array([0, 0])  # Starting position (x, y)
average_error = 0.5  # Average drift error per time step (units)
# max_error = 1.0  # Maximum drift error per time step (units)
pixels_per_meter = 51.2  # Conversion factor from meters to pixels
real_width = 30  # Width of the environment in meters
real_height = 30  # Height of the environment in meters
heatmap_size = (math.ceil(real_width * pixels_per_meter), math.ceil(real_height*pixels_per_meter))  # Size of the heatmap grid

#house
outside_area_bg = []#[(1.3592, 0.5182), (4.263, -1.876)]

def generate_heatmap(size, num_spots=20, min_spot_size = 30, max_spot_size=300, max_amplitude=1, min_amplitude=0):
    # Create a random array to store the heatmap values
    heatmap = np.zeros(size)
    
    # Generate random heat and cold spots
    for _ in range(num_spots):
        # Randomly select spot characteristics
        spot_center = (np.random.randint(0, size[0]), np.random.randint(0, size[1]))
        spot_size = np.random.randint(min_spot_size, max_spot_size)
        spot_amplitude = np.random.uniform(min_amplitude, max_amplitude)

        # Randomly stretch the Gaussian in different directions (elliptical distortion)
        stretch_x = np.random.uniform(0.2, 5)  # Stretch factor in X direction
        stretch_y = np.random.uniform(0.2, 5)  # Stretch factor in Y direction

        # Generate the distorted spot using a Gaussian distribution
        Y, X = np.ogrid[:size[0], :size[1]]
        distance = ((X - spot_center[1]) / stretch_x)**2 + ((Y - spot_center[0]) / stretch_y)**2
        spot = spot_amplitude * np.exp(-distance / (2 * (spot_size**2)))

        # Add the spot to the heatmap (making sure to clip the values to avoid overflow)
        heatmap += np.clip(spot, min_amplitude, max_amplitude)
    
    #iterate ofer the entire heatmap
    for i in range(size[0]):
        for j in range(size[1]):
            #check if the point is inside the house
            for area in outside_area_bg:
                if i/pixels_per_meter >= (area[0]+real_width/2) and j/pixels_per_meter >= (area[1]+real_height/2):
                    heatmap[i][j] = -1
    

    # Apply a lower amount of smoothing to preserve more peaks/valleys
    # heatmap = gaussian_filter(heatmap, sigma=1)  # lower sigma value

    # Normalize the heatmap to have values between 0 and 1
    heatmap = (heatmap - np.min(heatmap)) / (np.max(heatmap) - np.min(heatmap))

    # gamma_dist = np.random.gamma(gamma_shape, gamma_scale, size)
    gamma_dist = truncated_gamma(gamma_shape, gamma_scale, min_error, max_error, size)

    # Flatten the gamma distribution and heatmap for sorting
    gamma_flattened = gamma_dist.flatten()
    heatmap_flattened = heatmap.flatten()

    # Get indices to sort the heatmap
    heatmap_sort_indices = np.argsort(heatmap_flattened)

    # Sort the gamma distribution values
    sorted_gamma_values = np.sort(gamma_flattened)

    # Map sorted gamma values to the heatmap order
    reordered_gamma_flattened = np.zeros_like(gamma_flattened)
    reordered_gamma_flattened[heatmap_sort_indices] = sorted_gamma_values

    # Reshape back to 2D
    reordered_gamma_dist = reordered_gamma_flattened.reshape(size)

    # Replace the heatmap values with the reordered gamma distribution values
    heatmap = reordered_gamma_dist
    print("std:", np.std(heatmap))
    print("mean:", np.mean(heatmap))
    print("min:", np.min(heatmap))
    print("max:", np.max(heatmap))
    print("median",     np.median(heatmap))
    print(" ")
    print(gamma_shape)
    print(gamma_scale)
    new_gamma_shape = (np.mean(heatmap) **2) / (np.std(heatmap) **2)
    new_gamma_scale = (np.std(heatmap) **2) / np.mean(heatmap)
    print(new_gamma_shape)
    print(new_gamma_scale)

    print(np.max(heatmap))
    print(np.min(heatmap))


    return heatmap

# # # Define the size of the heatmap (height, width)
# size = (64, 64)  # 100x100 grid
# heatmap = generate_heatmap(size)

# # Plot the result
# plt.imshow(heatmap, cmap='coolwarm', interpolation='nearest')
# plt.colorbar(label='Heat intensity')
# plt.title("Generated Heatmap with Cold and Heat Spots")
# plt.show()



# Generate a directional heatmap with smooth variations
def generate_error_heatmap(size):
    """Create a heatmap where each point has a smooth, correlated random direction."""
    errors = np.zeros(size)
    num_sets = np.random.randint(5, 20)  # Random number of sets between 5 and 20
    total_area = size[0] * size[1]
    remaining_area = total_area
    sets = []
    
    # for _ in range(num_sets - 1):
    #     set_height = np.random.randint(5, size[0] + 1)
    #     set_width = np.random.randint(5, size[1] + 1)
    #     set_area = set_height * set_width
    #     if set_width > size[1]:
    #         set_width = size[1]
    #         set_height = set_area // set_width
    #     set_size = (set_height, set_width)
    #     set_errors = np.random.gamma(gamma_shape, gamma_scale, set_size)
    #     reordered_set = reorder_for_complex_mountains_and_valleys(set_errors)
        
    #     # Find a random position to place the set in the errors array
    #     start_row = np.random.randint(0, size[0] - set_size[0] + 1)
    #     start_col = np.random.randint(0, size[1] - set_size[1] + 1)
        
    #     errors[start_row:start_row + set_size[0], start_col:start_col + set_size[1]] = reordered_set
    #     remaining_area -= set_area
    
    # Fill the remaining part with the last set
    remaining_size = (size[0], size[1])
    # remaining_errors = np.random.gamma(gamma_shape, gamma_scale, remaining_size)
    errors = generate_heatmap(remaining_size)
    
    # Fill only the non-filled entries
    # mask = (errors == 0)
    # errors[mask] = reordered_remaining[mask]
    
    # errors = np.clip(errors, min_error, max_error)
    return errors



# Initialize variables
# errors = generate_error_heatmap(heatmap_size)

# # # #normalize errors
# errors = (errors - np.min(errors)) / (np.max(errors) - np.min(errors))


# # # # # # Save the directions heatmap to a file
# with open('/home/hugo/Documents/Thesis_ARGoS/error_heatmap_museum.txt', 'w') as f:
#     # f.write('{')
#     for row in errors:
#         f.write('{')
#         f.write(', '.join(map(str, row)))
#         f.write('},\n')
# #     # f.write('}\n')
# # positions = [initial_position]

# Read the directions heatmap from a file
errors = []
with open('/home/hugo/Documents/Thesis_ARGoS/implementat    ion_and_examples/controllers/pipuck_hugo/heatmaps/error_heatmap_museum.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
        if line.strip().startswith('{') and line.strip().endswith('},'):
            row = line.strip()[1:-2].split(', ')
            errors.append([float(val) for val in row])
errors = np.array(errors)

print("std:", np.std(errors))
print("mean:", np.mean(errors))
print("min:", np.min(errors))
print("max:", np.max(errors))
print("median",     np.median(errors))
print(" ")
print(gamma_shape)
print(gamma_scale)
new_gamma_shape = (np.mean(errors) **2) / (np.std(errors) **2)
new_gamma_scale = (np.std(errors) **2) / np.mean(errors)
print(new_gamma_shape)
print(new_gamma_scale)

print(np.max(errors))
print(np.min(errors))




# Visualize the directional heatmap using colors to represent angles
angle_colors = (errors / (max_error-min_error))  # Normalize angles to [0, 1] for coloring
# plt.figure(figsize=(8, 8))
# plt.imshow(angle_colors, cmap='Spectral_r', origin='lower')
#also plot the actual errors
plt.figure(figsize=(8, 8))
plt.imshow(errors.T, cmap='Spectral_r', origin='lower')
plt.colorbar(label='Error (normalized)')
plt.title('Error Magnitude Heatmap (Color-encoded)')
plt.xlabel('X Direction')
plt.ylabel('Y Direction')
plt.grid(True)

# # Add arrows representing directions
# x_indices, y_indices = np.meshgrid(np.arange(heatmap_size[0]), np.arange(heatmap_size[1]))
# arrows_x = np.cos(directions)
# arrows_y = np.sin(directions)
# plt.quiver(x_indices, y_indices, arrows_x, arrows_y, scale=30, color='black', alpha=0.6)
plt.show()
