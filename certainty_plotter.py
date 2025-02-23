import pandas as pd
import matplotlib.pyplot as plt
import os
import math

subplots = False
ticks_per_second = 16

if subplots:

    n_directories = len(os.listdir('implementation_and_examples/experiment_results/office_keep'))

    fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))

    #all in separate subplot

    # for all directories in 'experiment_results'
    for i, directory in enumerate(os.listdir('implementation_and_examples/experiment_results/office_keep')):
        # Construct the path to the CSV file in the current directory
        csv_path = os.path.join('implementation_and_examples/experiment_results/office_keep', directory, 'spawn_time_0/6_agents/certainty.csv')

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
    #all in one subplot
    fig, ax = plt.subplots(figsize=(12, 8))

    for i, directory in enumerate(os.listdir('implementation_and_examples/experiment_results/office_keep')):
        # Construct the path to the CSV file in the current directory
        csv_path = os.path.join('implementation_and_examples/experiment_results/office_keep', directory, 'spawn_time_0/6_agents/certainty.csv')

        # Check if the CSV file exists
        if os.path.exists(csv_path):
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
                        ax.plot(data['time_s']/ticks_per_second, data[column], label=f'{directory} - {column}')
                    # if column == 'free' or column == 'occupied':
                    #     ax.plot(data['time_s']/ticks_per_second, abs(data[column]-0.5), label=f'{directory} - {column}')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage')
    ax.set_title('Coverage Over Time for Each Agent')
    ax.legend()
    ax.grid(True)

    data_frames = [pd.read_csv(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv')) for directory in os.listdir('implementation_and_examples/experiment_results/office') if os.path.exists(os.path.join('implementation_and_examples/experiment_results/office', directory, 'spawn_time_0/6_agents/certainty.csv'))]
    max_time = max([data['time_s'].max() for data in data_frames])

    ax.set_xlim(0, 450)
    ax.set_ylim(0, 0.4)

    plt.tight_layout()
    plt.show()
