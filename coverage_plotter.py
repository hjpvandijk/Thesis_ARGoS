import pandas as pd
import matplotlib.pyplot as plt
import os

subplots = False
ticks_per_second = 16

path = 'implementation_and_examples/experiment_results/office_keep'

n_directories = len(os.listdir(path))

if subplots:
    fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))

    #for all directories in 'experiment_results'
    for i, directory in enumerate(os.listdir(path)):
        # Construct the path to the CSV file in the current directory
        csv_path = os.path.join(path, directory, 'spawn_time_0/6_agents/coverage.csv')

        # Check if the CSV file exists
        if os.path.exists(csv_path):
            # Read the CSV file
            data = pd.read_csv(csv_path)
            
            #calculate average
            data['average'] = data[[column for column in data.columns if column != 'time_s']].mean(axis=1)

            for column in data.columns[1:]:
                # if column == 'average':
                #     axs[i].plot(data['time_s']/ticks_per_second, data[column], label=column)
                #if column not unnamed
                if not column.startswith('Unnamed'):
                    axs[i].plot(data['time_s']/ticks_per_second, data[column], label=column)

            axs[i].set_xlabel('Time (s)')
            axs[i].set_ylabel('Coverage')
            axs[i].set_title(f'Coverage Over Time for Each Agent in {directory}')
            axs[i].legend()
            axs[i].grid(True)
    plt.tight_layout()
    plt.show()

else:
    #all in one subplot
    # for all directories in 'experiment_results'
    for directory in os.listdir(path):
        # Construct the path to the CSV file in the current directory
        csv_path = os.path.join(path, directory, 'spawn_time_0/6_agents/coverage.csv')

        # Check if the CSV file exists
        if os.path.exists(csv_path):
            # Read the CSV file
            data = pd.read_csv(csv_path)
            
            #calculate average
            data['average'] = data[[column for column in data.columns if column != 'time_s']].mean(axis=1)

            for column in data.columns[1:]:
                if column == 'average':
                    plt.plot(data['time_s']/ticks_per_second, data[column],label=f'{directory} - {column}')
                # if column not unnamed
                # if not column.startswith('Unnamed'):
                #     plt.plot(data['time_s']/ticks_per_second, data[column], label=column)

    plt.xlabel('Time (s)')
    plt.ylabel('Coverage')
    plt.title('Coverage Over Time for Each Agent')
    plt.xlim(0, 450)
    plt.legend()
    plt.grid(True)
    plt.show()