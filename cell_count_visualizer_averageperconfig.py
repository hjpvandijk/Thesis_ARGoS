import pandas as pd
import matplotlib.pyplot as plt
import os
import math
from matplotlib.widgets import CheckButtons
import numpy as np


subplots = False
ticks_per_second = 16

path = 'implementation_and_examples/experiment_results'

#n configs
n_directories = 4

n_agent_configs = 9 #2-10 agents

map_max = {
    'museum': {
        'vmax': 621,
        'cell_count_x': 650,
        'cell_count_y': 10**4
    },
    'museum_tilted': {
        'vmax': 621,
        'cell_count_x': 200,
        'cell_count_y': 10**5
    },
    'office': {
        'vmax': 1463,
        'cell_count_x': 1400,
        'cell_count_y': 10**4
    },
    'office_tilted': {
        'vmax': 1300,
        'cell_count_x': 1200,
        'cell_count_y': 10**4
    },
    'house': {
        'vmax': 1724,
        'cell_count_x': 1750,
        'cell_count_y': 10**3
    },
    'house_tilted': {
        'vmax': 1792,
        'cell_count_x': 2700,
        'cell_count_y': 10**3
    }
}


for i, map in enumerate(os.listdir(path)):
    if map != 'museum_tilted':
        continue

    for j, config in enumerate(os.listdir(os.path.join(path, map))):
        all_visits = pd.DataFrame()
        n_succesful_agents = 9
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                # Construct the path to the CSV file in`` the current directory
                csv_path = os.path.join(path, map, config, spawn_time, agents, 'map_observation_count.csv')

                # Check if the CSV file exists
                if os.path.exists(csv_path):
                    #if csv file empty, skip
                    # if os.path.getsize(csv_path) == 0:
                    #     n_succesful_agents -= 1
                    #     continue
                    
                    data = pd.read_csv(csv_path)

                    #if no x and y in all_visits, add it
                    if 'x' not in all_visits.columns:
                        all_visits['x'] = data['x']
                        all_visits['y'] = data['y']
                        all_visits['observation_count_average'] = 0

                    # values = [entry for entry in data.columns if entry not in ['x', 'y']]

                    #divide observation count by number of agents
                    data['observation_count_average'] = data['observation_count_total'] / int(agents.split('_')[0])
                    
                    #sum this data to all_visits
                    all_visits['observation_count_average'] += data['observation_count_average']


        #average all_visits
        all_visits['observation_count_average'] = all_visits['observation_count_average'] / n_succesful_agents

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 14))

        # Pivot the data to create a matrix for the heatmap
        heatmap_data = all_visits.pivot(index='y', columns='x', values='observation_count_average')
        # Create a mask for cells with value 0
        mask = np.zeros_like(heatmap_data, dtype=bool)
        mask[heatmap_data == 0] = True

        # Plot the heatmap with the mask
        masked_heatmap_data = np.ma.masked_where(mask, heatmap_data)
        heatmap = ax1.imshow(masked_heatmap_data, cmap='Spectral_r', origin='lower', vmax=map_max[map]['vmax'])
        # plt.imshow(mask, cmap='gray', origin='lower', alp ha=0.5)  # Overlay the mask with black color
        fig.colorbar(mappable=heatmap, ax=ax1, label='Count')

        ax1.set_title('Observation Count Heatmap: ' + map + ' - ' + config + ' - ' + str(n_succesful_agents) + ' agents')
        ax1.set_xlabel('X-axis')
        ax1.set_ylabel('Y-axis')


        # Count the number of cells visited 1, 2, 3, ... times
        visit_counts = heatmap_data.stack().value_counts().sort_index()
        # Disregard the 0 value
        visit_counts = visit_counts[1:]
        # Increase until 1000
        while visit_counts.index.max() < map_max[map]['cell_count_x']:
            visit_counts = visit_counts.append(pd.Series([0], index=[visit_counts.index.max() + 1]))

        # Group the visit counts into intervals of 50
        intervals = np.arange(0, visit_counts.index.max() + 50, 50)
        visit_counts_grouped = visit_counts.groupby(pd.cut(visit_counts.index, intervals)).sum()

        # Plot the bar graph
        ax2.bar(visit_counts_grouped.index.astype(str), visit_counts_grouped.values)
        ax2.set_title('Number of Cells Observed: ' + map + ' - ' + config + ' - ' + str(n_succesful_agents) + ' agents')
        ax2.tick_params(axis='x', rotation=45)  # Rotate x-axis labels for better readability
        ax2.set_xlabel('Number of Visits')
        ax2.set_ylabel('Number of Cells')
        ax2.set_yscale('symlog')
        ax2.set_ylim(0, map_max[map]['cell_count_y'])
        ax2.grid(True)

    plt.show()
