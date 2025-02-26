import pandas as pd
import matplotlib.pyplot as plt
import os

subplots = False
ticks_per_second = 16

map = 'museum'

if map == 'museum':
    valid_area = 697.082520

path = 'implementation_and_examples/experiment_results/museum/config__alignment0__cohesion__0_1/spawn_time_0'

n_directories = len(os.listdir(path))

if subplots:
    fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))

    #for all directories in 'experiment_results'
    for i, directory in enumerate(os.listdir(path)):
        # Construct the path to the CSV file in the current directory
        csv_path = os.path.join(path, directory, 'coverage.csv')

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
        csv_path = os.path.join(path, directory, 'coverage.csv')

        # Check if the CSV file exists
        if os.path.exists(csv_path):
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


            for column in data.columns[1:]:
                if column == 'average':
                    #add 0 at time 0
                    plt.plot(data['time_s']/ticks_per_second, data[column]/valid_area * 100,label=f'{directory} - {column}')
                # if column not unnamed
                # if not column.startswith('Unnamed'):
                #     plt.plot(data['time_s']/ticks_per_second, data[column], label=column)
    #order the legend
    handles, labels = plt.gca().get_legend_handles_labels()
    sorted_handles = [x for _, x in sorted(zip(labels, handles),key=lambda s: int(s[0].split('_')[0]))]
    #make the colors go from lighter to darker
    for i, handle in enumerate(sorted_handles):
        handle.set_color(plt.cm.viridis(1 - i/len(sorted_handles)))

    sorted_labels = sorted(labels,key=lambda s: int(s.split('_')[0]))
    #order alphabetically
    plt.legend(sorted_handles, sorted_labels)
    plt.xlabel('Time (s)')
    plt.ylabel('Coverage (%)')
    plt.title('Coverage Over Time for Each Agent')
    # plt.xlim(0, 450)
    #set x in steps of 10
    plt.yticks(range(0, 110, 10))
    plt.grid(True)
    plt.show()