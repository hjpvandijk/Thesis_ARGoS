import pandas as pd
import matplotlib.pyplot as plt
import os
import math
from matplotlib.widgets import CheckButtons

subplots = False
ticks_per_second = 16

path = 'implementation_and_examples/experiment_results'

#n configs
n_directories = 4

map_max = {
    'museum': 500,
    'office': 180,
    'house': 80
    
}


for i, map in enumerate(os.listdir(path)):
    fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))
    for j, config in enumerate(os.listdir(os.path.join(path, map))):
        ax = axs[j] if n_directories > 1 else axs
        average = []
        time = []
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                # Construct the path to the CSV file in the current directory
                csv_path = os.path.join(path, map, config, spawn_time, agents, 'coverage.csv')

                # Check if the CSV file exists
                if os.path.exists(csv_path):
                    #if csv file empty, skip
                    if os.path.getsize(csv_path) == 0:
                        continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path)
                    
                   #if some columns are longer than others, repeat the last entry of the shorter columns
                    data.fillna(method='ffill', inplace=True)
                    # data.fillna(0, inplace=True)

                    
                    #calculate average
                    data['average'] = data[[column for column in data.columns if column != 'time_s']].mean(axis=1)
                    #add row of 0 to the front
                    data.loc[-1] = [0] + [0 for _ in range(len(data.columns)-1)]
                    data.index = data.index + 1
                    data = data.sort_index()

                    average.append(data['average'])
                    # times = data['time_s']/ticks_per_second
                    if len(data['time_s']) > len(time):
                        time = data['time_s']/ticks_per_second

                    # for column in data.columns[1:]:
                    #     # if column not unnamed
                    #     if not column.startswith('Unnamed'):
                    #         if column == 'average':
                    #             # ax.plot(data['time_s']/ticks_per_second, data[column], label=f'{map} - {config} - {spawn_time} - {agents} - {column}')
                    #             ax.plot(data['time_s']/ticks_per_second, data[column], label=f'{agents} - {column}')
                    #         # if column == 'free' or column == 'occupied':
                    #         #     ax.plot(data['time_s']/ticks_per_second, abs(data[column]-0.5), label=f'{directory} - {column}')
        #average over all rows in 'average'
        average_df = pd.DataFrame(average)
        average_df.fillna(method='ffill', inplace=True)
        s = average_df.mean(axis=0)
        #divide by number of rows
        # s = [x/len(average) for x in s]
        ax.plot(time, s, label='average')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Coverage')
        ax.set_title(f'Coverage Over Time for {map} - {config}')
        ax.grid(True)
        map_name = map.split('_')[0]
        ax.set_yticks(range(0, map_max[map_name], 10))
        ax.set_xlim(0, 400)


        #order the legend
        handles, labels = ax.get_legend_handles_labels()
        # Extract the number of agents from the labels and sort accordingly
        def extract_agents(label):
            if label == 'average':
                return 0
            return int(label.split(' - ')[0].split('_')[0])

        sorted_handles = [x for _, x in sorted(zip(labels, handles), key=lambda pair: extract_agents(pair[0]))]
        #make the colors go from lighter to darker
        for i, handle in enumerate(sorted_handles):
            handle.set_color(plt.cm.viridis(1 - i/len(sorted_handles)))

        sorted_labels = sorted(labels, key=extract_agents)
        ax.legend(sorted_handles, sorted_labels)

        # Create a list of all unique agent labels
        all_labels = set()
        for ax in axs if n_directories > 1 else [axs]:
            handles, labels = ax.get_legend_handles_labels()
            all_labels.update(labels)

        # Sort the labels the same way as the legend
        sorted_all_labels = sorted(all_labels, key=extract_agents)

        # Create a CheckButtons widget
        rax = plt.axes([0.91, 0.4, 0.1, 0.1])
        check = CheckButtons(rax, sorted_all_labels, [True] * len(sorted_all_labels))

        # Define a function to toggle visibility of lines
        def toggle_visibility(label):
            for ax in axs if n_directories > 1 else [axs]:
                handles, labels = ax.get_legend_handles_labels()
                for handle, lbl in zip(handles, labels):
                    if lbl == label:
                        handle.set_visible(not handle.get_visible())
            plt.draw()

        # Connect the CheckButtons to the toggle function
        check.on_clicked(toggle_visibility)
    plt.show()
