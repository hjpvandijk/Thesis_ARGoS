import pandas as pd
import matplotlib.pyplot as plt
import os
import math

subplots = False
ticks_per_second = 16

path = 'implementation_and_examples/experiment_results'


if subplots:

    n_directories = len(os.listdir(path))

    fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))

    #all in separate subplot

    # for all directories in 'experiment_results'
    for i, directory in enumerate(os.listdir(path)):
        # Construct the path to the CSV file in the current directory
        csv_path = os.path.join(path, directory, 'certainty.csv')

        # Check if the CSV file exists
        if os.path.exists(csv_path):
            # Read the CSV file
            data = pd.read_csv(csv_path)
            
            # average certainty of all columnst starting with 'all
            data['all'] = data[[column for column in data.columns if column.startswith('all')]].mean(axis=1)

            for column in data.columns[1:]:
                #if column not unnamed
                if not column.startswith('Unnamed'):
                    if column == 'all':
                        axs[i].plot(data['time_s']/ticks_per_second, data[column], label=column)
                    # if column.startswith('all'):
                    #     axs[i].plot(data['time_s']/ticks_per_second, data[column], label=column)
                    # else:
                    #     axs[i].plot(data['time_s']/ticks_per_second, abs(data[column]-0.5), label=column)

            axs[i].set_xlabel('Time (s)')
            axs[i].set_ylabel('Coverage')
            axs[i].set_title(f'Coverage Over Time for Each Agent in {directory}')
            axs[i].legend()
            axs[i].grid(True)



    data_frames = [pd.read_csv(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv')) for directory in os.listdir('implementation_and_examples/experiment_results/office') if os.path.exists(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv'))]
    max_time = max([data['time_s'].max() for data in data_frames])

    for ax in axs:
        ax.set_xlim(0, 450)
        ax.set_ylim(0, 0.4)
        # ax.set_xticks(range(0, int(max_time/ticks_per_second), 10))

    plt.tight_layout()
    plt.show()

else:


    for i, map in enumerate(os.listdir(path)):
        for j, config in enumerate(os.listdir(os.path.join(path, map))):
            if config != 'n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5':
                continue
            for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
                for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                    if not (4 <= int(agents.split('_')[0]) <= 8):
                        continue
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
                                    plt.plot(data['tick']/ticks_per_second, data[column], label=f'{map} - {config} - {spawn_time} - {agents} - {column}')
                                # if column == 'free' or column == 'occupied':
                                #     ax.plot(data['time_s']/ticks_per_second, abs(data[column]-0.5), label=f'{directory} - {column}')

    plt.xlabel('Time (s)')
    plt.ylabel('Certainty')
    plt.title('Certainty Over Time for Each Agent')
    plt.grid(True)

    #order the legend
    handles, labels = plt.gca().get_legend_handles_labels()
    sorted_handles = [x for _, x in sorted(zip(labels, handles))]
    #make the colors go from lighter to darker
    for i, handle in enumerate(sorted_handles):
        handle.set_color(plt.cm.viridis(1 - i/len(sorted_handles)))

    sorted_labels = sorted(labels)
    plt.legend(sorted_handles, sorted_labels)

    # data_frames = [pd.read_csv(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv')) for directory in os.listdir('implementation_and_examples/experiment_results/office') if os.path.exists(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv'))]
    # max_time = max([data['tick'].max() for data in data_frames])

    # plt.xlim(0, 450)
    plt.ylim(0, 0.4)

    plt.tight_layout()
    plt.show()
