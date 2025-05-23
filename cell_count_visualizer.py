import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read the CSV file
# data = pd.read_csv('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_0_wifi_range_99999_message_loss_probability_0_frontier_search_radius_99999_max_frontier_regions_99999_evaporation_time_100_max_route_length_99999/spawn_time_0/2_agents/S1/map_observation_count.csv')
data = pd.read_csv('/home/hugo/Documents/experiment_results/house/AAVFIX_end_time_400_noise_0_wifi_range_99999_message_loss_probability_0_frontier_search_radius_99999_max_frontier_regions_99999_evaporation_time_100_max_route_length_99999/spawn_time_0/2_agents/S1/map_observation_count.csv')

#get the column titles
values = [entry for entry in data.columns if entry not in ['x', 'y']]
# Pivot the data to create a matrix for the heatmap
heatmap_data = data.pivot(index='y', columns='x', values='observation_count_total')

# Plot the heatmap
plt.figure(figsize=(10, 8))
# Create a mask for cells with value 0
mask = np.zeros_like(heatmap_data, dtype=bool)
mask[heatmap_data == 0] = True

# Plot the heatmap with the mask
masked_heatmap_data = np.ma.masked_where(mask, heatmap_data)
heatmap = plt.imshow(masked_heatmap_data, cmap='Spectral', origin='lower', vmax=2000)
# plt.imshow(mask, cmap='gray', origin='lower', alp ha=0.5)  # Overlay the mask with black color
plt.colorbar(mappable=heatmap, label='Count')


plt.title('Observation Count Heatmap')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

# Count the number of cells visited 1, 2, 3, ... times
visit_counts = heatmap_data.stack().value_counts().sort_index()
# Disregard the 0 value
visit_counts = visit_counts[1:]

# Group the visit counts into intervals of 250
intervals = np.arange(0, visit_counts.index.max() + 100, 100)
visit_counts_grouped = visit_counts.groupby(pd.cut(visit_counts.index, intervals)).sum()

# Plot the line graph
plt.figure(figsize=(10, 6))
plt.plot(visit_counts_grouped.index.astype(str), visit_counts_grouped.values, marker='o')
plt.title('Number of Cells Observed')
plt.xticks(rotation=45, ha='right')  # Rotate x-axis labels for better readability
plt.xlabel('Number of Visits')
plt.ylabel('Number of Cells')
plt.grid(True)
plt.show()