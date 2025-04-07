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
    'house': 100
}

n_agents_list = []


for i, map in enumerate(os.listdir(path)):
    # if map not in ['museum', 'office_tilted', 'house_tilted']:
    #     continue
    # fig,axs = plt.subplots(n_directories, 1, figsize=(12, 8*n_directories))
    # ax = axs[j] if n_directories > 1 else axs
    fig,ax1 = plt.subplots()
    color = 'tab:red'
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Certainty', color=color)
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    color = 'tab:blue'
    ax2.set_ylabel('Coverage', color=color)
    ax2.tick_params(axis='y', labelcolor=color)
    fig.tight_layout()

    for j, config in enumerate(os.listdir(os.path.join(path, map))):
        # if config not in ['config_bigger_safety_range', 'config_bigger_safety_range_smaller_n', 'config_bigger_safety_n_1', ]:
        if config not in ['p_sensor_1', 'p_sensor_0_9', 'p_sensor_0_75', 'p_sensor_0_5']:
            continue
        average_all = []
        average_free = []
        average_occupied = []
        average_coverage = []
        time_certainty = []
        time_coverage = []
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                # Construct the path to the CSV file in the current directory
                csv_path_certainty = os.path.join(path, map, config, spawn_time, agents, 'certainty.csv')
                csv_path_coverage = os.path.join(path, map, config, spawn_time, agents, 'coverage.csv')

                # Check if the CSV file exists
                if os.path.exists(csv_path_certainty):
                    n_agents_list.append(agents.split('_')[0])
                    #if csv file empty, skip
                    if os.path.getsize(csv_path_certainty) == 0:
                        continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path_certainty)
                    
                   #if some columns are longer than others, repeat the last entry of the shorter columns
                    data.fillna(method='ffill', inplace=True)
                    # data.fillna(0, inplace=True)
                    
                    
                    data['all'] = data[[column for column in data.columns if column.startswith('all')]].mean(axis=1)                        

                    
                   # average certainty of all columns starting with 'all'
                    data['all'] = data[[column for column in data.columns if column.startswith('all')]].mean(axis=1)                        

                    # average certainty of all columns starting with 'free'
                    data['free'] = data[[column for column in data.columns if column.startswith('free')]].mean(axis=1)
                    # average certainty of all columns starting with 'occupied'
                    data['occupied'] = data[[column for column in data.columns if column.startswith('occupied')]].mean(axis=1)
                

                    average_all.append(data['all'])
                    average_free.append(data['free'])
                    average_occupied.append(data['occupied'])
                    # times = data['time_s']/ticks_per_second
                    if len(data['tick']) > len(time_certainty):
                        time_certainty = data['tick']/ticks_per_second

                    # for column in data.columns[1:]:
                    #     # if column not unnamed
                    #     if not column.startswith('Unnamed'):
                    #         if column == 'average':
                    #             # ax.plot(data['tick']/ticks_per_second, data[column], label=f'{map} - {config} - {spawn_time} - {agents} - {column}')
                    #             ax.plot(data['tick']/ticks_per_second, data[column], label=f'{agents} - {column}')
                    #         # if column == 'free' or column == 'occupied':
                    #         #     ax.plot(data['tick']/ticks_per_second, abs(data[column]-0.5), label=f'{directory} - {column}')
                # Check if the CSV file exists
                if os.path.exists(csv_path_coverage):
                    #if csv file empty, skip
                    if os.path.getsize(csv_path_coverage) == 0:
                        continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path_coverage)
                    
                   #if some columns are longer than others, repeat the last entry of the shorter columns
                    data.fillna(method='ffill', inplace=True)
                    # data.fillna(0, inplace=True)

                    
                    #calculate average
                    data['average'] = data[[column for column in data.columns if column != 'tick']].mean(axis=1)
                    #add row of 0 to the front
                    data.loc[-1] = [0] + [0 for _ in range(len(data.columns)-1)]
                    data.index = data.index + 1
                    data = data.sort_index()

                    average_coverage.append(data['average'])
                    # times = data['time_s']/ticks_per_second
                    if len(data['tick']) > len(time_coverage):
                        time_coverage = data['tick']/ticks_per_second

                    # for column in data.columns[1:]:
                    #     # if column not unnamed
                    #     if not column.startswith('Unnamed'):
                    #         if column == 'average':
                    #             # ax.plot(data['tick']/ticks_per_second, data[column], label=f'{map} - {config} - {spawn_time} - {agents} - {column}')
                    #             ax.plot(data['tick']/ticks_per_second, data[column], label=f'{agents} - {column}')
                    #         # if column == 'free' or column == 'occupied':
                    #         #     ax.plot(data['tick']/ticks_per_second, abs(data[column]-0.5), label=f'{directory} - {column}')
        #average over all rows in 'average'
        # average_free_df = pd.DataFrame(average_free)
        # average_free_df.fillna(method='ffill', inplace=True)
        # s_free = average_free_df.mean(axis=0)
        # #divide by number of rows
        # # s = [x/len(average) for x in s]
        # ax1.plot(time, s_free, label=f'certainty: free - {config}')

        # average_occupied_df = pd.DataFrame(average_occupied)
        # average_occupied_df.fillna(method='ffill', inplace=True)
        # s_occupied = average_occupied_df.mean(axis=0)
        # #divide by number of rows
        # # s = [x/len(average) for x in s]
        # ax1.plot(time, s_occupied, label=f'certainty: occupied - {config}')

        average_all_df = pd.DataFrame(average_all)
        average_all_df.fillna(method='ffill', inplace=True)
        s_all = average_all_df.mean(axis=0)
        #divide by number of rows
        # s = [x/len(average) for x in s]
        ax1.plot(time_certainty, s_all, label=f'{config}: certainty-all')

        average_df_coverage = pd.DataFrame(average_coverage)
        average_df_coverage.fillna(method='ffill', inplace=True)
        s_coverage = average_df_coverage.mean(axis=0)
        #divide by number of rows
        # s = [x/len(average) for x in s]
        ax2.plot(time_coverage, s_coverage, label=f'{config}: coverage')
        
    n_agents_list = list(set(n_agents_list))

    # plt.xlabel('Time (s)')
    # plt.ylabel('Certainty')
    plt.title(f'Certainty and Certainty Over Time for {map}, with agents {n_agents_list}')
    plt.grid(True)
    map_name = map.split('_')[0]
    ax1.set_ylim(0, 0.4)
    ax2.set_ylim(0, map_max[map_name])
    plt.xlim(0, 400)
    # plt.ylim(0,   .4)
    # plt.legend()


    #order the legend
    handles1, labels1 = ax1.get_legend_handles_labels()

    sorted_handles1 = [x for _, x in sorted(zip(labels1, handles1), key=lambda pair: pair[0].split(' - ')[0])]
    # #make the colors go from lighter to darker
    for i, handle in enumerate(sorted_handles1):
        handle.set_color(plt.cm.viridis(1 - i/len(sorted_handles1)))

    sorted_labels1 = sorted(labels1, key=lambda label: label.split(' : ')[0])
    ax1.legend(sorted_handles1, sorted_labels1, loc='lower left')
    
    handles2, labels2 = ax2.get_legend_handles_labels()

    sorted_handles2 = [x for _, x in sorted(zip(labels2, handles2), key=lambda pair: pair[0].split(' - ')[0])]
    # #make the colors go from lighter to darker
    for i, handle in enumerate(sorted_handles2):
        handle.set_color(plt.cm.viridis(1 - i/len(sorted_handles2)))
        handle.set_linestyle('--')

    sorted_labels2 = sorted(labels2, key=lambda label: label.split(' : ')[0])
    ax2.legend(sorted_handles2, sorted_labels2, loc='lower right')


    # plt.legend(sorted_handles, sorted_labels)

    # # Create a list of all unique agent labels
    # all_labels = set()

    # handles, labels = plt.gca().get_legend_handles_labels()
    # all_labels.update(labels)

    # # Sort the labels the same way as the legend
    # sorted_all_labels = sorted(all_labels, key=lambda label: label.split(' - ')[0])

    # # Create a CheckButtons widget
    # rax = plt.axes([0.91, 0.4, 0.1, 0.1])
    # check = CheckButtons(rax, sorted_all_labels, [True] * len(sorted_all_labels))

    # # # Define a function to toggle visibility of lines
    # def toggle_visibility(label):
    #     handles, labels = plt.gca().get_legend_handles_labels()
    #     for handle, lbl in zip(handles, labels):
    #         if lbl == label:
    #             handle.set_visible(not handle.get_visible())
    #     plt.draw()

    # # Connect the CheckButtons to the toggle function
    # check.on_clicked(toggle_visibility)
plt.show()
