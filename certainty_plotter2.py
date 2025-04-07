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



for i, map in enumerate(os.listdir(path)):
    fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))
    for j, config in enumerate(os.listdir(os.path.join(path, map))):
        ax = axs[j] if n_directories > 1 else axs
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                # Construct the path to the CSV file in the current directory
                csv_path = os.path.join(path, map, config, spawn_time, agents, 'certainty.csv')

                # Check if the CSV file exists
                if os.path.exists(csv_path):
                    #if csv file empty, skip
                    if os.path.getsize(csv_path) == 0:
                        continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path)
                    
                    # average certainty of all columns starting with 'all'
                    data['all'] = data[[column for column in data.columns if column.startswith('all')]].mean(axis=1)                        

                    # average certainty of all columns starting with 'free'
                    data['free'] = data[[column for column in data.columns if column.startswith('free')]].mean(axis=1)
                    # average certainty of all columns starting with 'occupied'
                    data['occupied'] = data[[column for column in data.columns if column.startswith('occupied')]].mean(axis=1)
                
                    for column in data.columns[1:]:
                        # if column not unnamed
                        if not column.startswith('Unnamed'):
                            if column == 'all':
                                # ax.plot(data['time_s']/ticks_per_second, data[column], label=f'{map} - {config} - {spawn_time} - {agents} - {column}')
                                ax.plot(data['time_s']/ticks_per_second, data[column], label=f'{agents} - {column}')
                            # if column == 'free' or column == 'occupied':
                            #     ax.plot(data['time_s']/ticks_per_second, abs(data[column]-0.5), label=f'{directory} - {column}')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Certainty')
        ax.set_title(f'Certainty Over Time for {map} - {config}')
        ax.grid(True)
        ax.set_ylim(0, 0.4)

        #order the legend
        handles, labels = ax.get_legend_handles_labels()
        # Extract the number of agents from the labels and sort accordingly
        def extract_agents(label):
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

    # plt.xlabel('Time (s)')
    # plt.ylabel('Certainty')
    # # plt.title('Certainty Over Time for Each Agent')
    # plt.grid(True)

    # #order the legend
    # handles, labels = plt.gca().get_legend_handles_labels()
    # sorted_handles = [x for _, x in sorted(zip(labels, handles))]
    # #make the colors go from lighter to darker
    # for i, handle in enumerate(sorted_handles):
    #     handle.set_color(plt.cm.viridis(1 - i/len(sorted_handles)))

    # sorted_labels = sorted(labels)
    # plt.legend(sorted_handles, sorted_labels)

    # data_frames = [pd.read_csv(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv')) for directory in os.listdir('implementation_and_examples/experiment_results/office') if os.path.exists(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv'))]
    # max_time = max([data['time_s'].max() for data in data_frames])

    # plt.xlim(0, 450)
    # plt.ylim(0, 0.4)

    # plt.tight_layout()
    # plt.title('Certainty Over Time for Each Agent: ' + map)
    plt.show()
