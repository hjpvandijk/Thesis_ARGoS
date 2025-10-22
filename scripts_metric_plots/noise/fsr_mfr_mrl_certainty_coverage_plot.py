from zipfile import ZipFile, Path
import pandas as pd
import os
import re
import csv
import matplotlib.pyplot as plt
from functools import lru_cache
import numpy as np
import time
import hashlib
import colorsys
from tabulate import tabulate

ticks_per_second = 16
prefix = ''

# batch = 'noise'

def parse_experiment_string(s):
    if not s.startswith('end'):
        end_index = s.index('end')
        global prefix
        prefix = s[:end_index]
        s = s[end_index:]
    # Regular expression to match key-value pairs
    pattern = r'([a-zA-Z_]+)_(-?\d+(?:_\d+)?)'
    matches = re.findall(pattern, s)

    # Convert values: replace underscores in numbers with a decimal point
    parsed_config = {key: float(value.replace('_', '.')) if '_' in value else int(value) for key, value in matches}
    
    end_time = parsed_config['end_time']
    noise = parsed_config['_noise']
    comm_range = parsed_config['_wifi_range']
    message_loss_probability = parsed_config['_message_loss_probability']
    frontier_search_radius = parsed_config['_frontier_search_radius']
    evaporation_time = parsed_config['_evaporation_time']
    max_frontier_regions = parsed_config['_max_frontier_regions']
    max_route_length = parsed_config['_max_route_length']
    
    return end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_regions, max_route_length

def create_experiment_string(
    end_time, noise, comm_range, message_loss_probability,
    frontier_search_radius, evaporation_time, max_frontier_regions, max_route_length
):
    # Convert float values back to the underscore format (e.g., 0.5 → 0_5)
    def format_value(value):
        return str(value).replace('.', '_') if isinstance(value, float) else str(value)
    
    # Construct the string
    config_string = (
        f"end_time_{format_value(end_time)}"
        f"_noise_{format_value(noise)}"
        f"_wifi_range_{format_value(comm_range)}"
        f"_message_loss_probability_{format_value(message_loss_probability)}"
        f"_frontier_search_radius_{format_value(frontier_search_radius)}"
        f"_evaporation_time_{format_value(evaporation_time)}"
        f"_max_frontier_regions_{format_value(max_frontier_regions)}"
        f"_max_route_length_{format_value(max_route_length)}"
    )
    
    return config_string

def get_unique_experiments_from_zip(configs):
    # print(configs)
    completed_unique = {}
    #read the csv files
    for file in os.listdir('completed_in_zip'):
        if file.endswith(".csv") and file.startswith("completed_experiments"):
            with open('completed_in_zip/' + file, 'r') as csvfile:
                if 'tilted' in file:
                        map = file.split('_')[-2].split('.')[0] + '_tilted'
                else:
                    map = file.split('_')[-1].split('.')[0]
                if map not in completed_unique:
                    completed_unique[map] = []
                reader = csv.reader(csvfile)
                # next(reader) #to skip potential header
                for row in reader:
                    config = row[0].split('/')[1]
                    if config not in configs:
                        continue
                    completed_unique[map].append(row[0])
    return completed_unique                    

@lru_cache(maxsize=100)  # Cache up to 100 files
def read_file(filename):
    with open(filename, "r") as f:
        return list(csv.reader(f)) # Load the entire file into memory
    
def get_values_for_each_category(configs):
    completed_experiments = get_unique_experiments_from_zip(configs)
    categories_and_values = {}
    for map, map_exp in completed_experiments.items():
        if map not in categories_and_values:
            categories_and_values[map] = {}
        for exp in map_exp:
            split_exp = exp.split('/')
            config = split_exp[1]
            end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_regions, max_route_length = parse_experiment_string(config)
            spawn_time = split_exp[2]
            agents = split_exp[3]
            seed = split_exp[4]

            if 'agents' not in categories_and_values[map]:
                categories_and_values[map]['agents'] = set()
            if 'spawn_time' not in categories_and_values[map]:
                categories_and_values[map]['spawn_time'] = set()
            if 'seed' not in categories_and_values[map]:
                categories_and_values[map]['seed'] = set()
            if 'end_time' not in categories_and_values[map]:
                categories_and_values[map]['end_time'] = set()
            if 'noise' not in categories_and_values[map]:
                categories_and_values[map]['noise'] = set()
            if 'comm_range' not in categories_and_values[map]:
                categories_and_values[map]['comm_range'] = set()
            if 'message_loss_probability' not in categories_and_values[map]:
                categories_and_values[map]['message_loss_probability'] = set()
            if 'frontier_search_radius' not in categories_and_values[map]:
                categories_and_values[map]['frontier_search_radius'] = set()
            if 'evaporation_time' not in categories_and_values[map]:
                categories_and_values[map]['evaporation_time'] = set()
            if 'max_frontier_regions' not in categories_and_values[map]:
                categories_and_values[map]['max_frontier_regions'] = set()
            if 'max_route_length' not in categories_and_values[map]:
                categories_and_values[map]['max_route_length'] = set()

            categories_and_values[map]['agents'].add(agents)
            categories_and_values[map]['spawn_time'].add(spawn_time)
            categories_and_values[map]['seed'].add(seed)
            categories_and_values[map]['end_time'].add(end_time)
            categories_and_values[map]['noise'].add(noise)
            categories_and_values[map]['comm_range'].add(comm_range)
            categories_and_values[map]['message_loss_probability'].add(message_loss_probability)
            categories_and_values[map]['frontier_search_radius'].add(frontier_search_radius)
            categories_and_values[map]['evaporation_time'].add(evaporation_time)
            categories_and_values[map]['max_frontier_regions'].add(max_frontier_regions)
            categories_and_values[map]['max_route_length'].add(max_route_length)
    return completed_experiments, categories_and_values

def check_if_all_required_experiments_done(completed_experiments, categories_and_values):
    #for all combinations check which configs are not in the completed experiments
    n_completed = 0
    n_non_completed = 0
    for map, map_values in categories_and_values.items():
        end_times = map_values['end_time']
        noises = sorted(map_values['noise'])
        comm_ranges = sorted(map_values['comm_range'])
        message_loss_probabilities = sorted(map_values['message_loss_probability'])
        frontier_search_radii = sorted(map_values['frontier_search_radius'])
        evaporation_times = sorted(map_values['evaporation_time'])
        max_frontier_regions = sorted(map_values['max_frontier_regions'])
        max_route_lengths = sorted(map_values['max_route_length'])
        agents = sorted(map_values['agents'], key=lambda x: int(x.split('_')[0]))
        spawn_times = sorted(map_values['spawn_time'])
        seeds = sorted(map_values['seed'])
        for end_time in end_times:
            for noise in noises:
                for comm_range in comm_ranges:
                    for message_loss_probability in message_loss_probabilities:
                        for frontier_search_radius in frontier_search_radii:
                            for evaporation_time in evaporation_times:
                                for max_frontier_region in max_frontier_regions:
                                    for max_route_length in max_route_lengths:
                                        for agent in agents:
                                            for spawn_time in spawn_times:
                                                for seed in seeds:
                                                    config = create_experiment_string(
                                                        end_time, noise, comm_range, message_loss_probability,
                                                        frontier_search_radius, evaporation_time, max_frontier_region, max_route_length
                                                    )
                                                    exp = f'{map}/{prefix}{config}/{spawn_time}/{agent}/{seed}'
                                                    if exp not in completed_experiments[map]:
                                                        print("not completed: ", exp)
                                                        n_non_completed += 1
                                                    else:
                                                        n_completed += 1
        print("MAP:", map)
        print("end_times:", end_times)
        print("seeds:", seeds)
        print("agents:", agents)
        print("spawn_times:", spawn_times)
        print("noises:", noises)
        print("comm_ranges:", comm_ranges)
        print("message_loss_probabilities:", message_loss_probabilities)
        print("frontier_search_radii:", frontier_search_radii)
        print("evaporation_times:", evaporation_times)
        print("max_frontier_regions:", max_frontier_regions)
        print("max_route_lengths:", max_route_lengths)
        print()
        time.sleep(1)
    print(f"Completed: {n_completed}, non-completed: {n_non_completed}")


@lru_cache(maxsize=20)  # Adjust cache size as needed
def get_zip(zip_path):
    return ZipFile(zip_path, "r")

def generate_high_contrast_hsv_colors(n_colors):
    hues = np.linspace(0, 1, n_colors, endpoint=False)  # Spread hues
    saturations = np.tile([1.0, 0.8], n_colors // 2 + 1)[:n_colors]  # Alternate saturation
    values = np.tile([1.0, 0.6], n_colors // 2 + 1)[:n_colors]  # Alternate brightness

    return [colorsys.hsv_to_rgb(hues[i], saturations[i], values[i]) for i in range(n_colors)]

def plot_certainty_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/certainty_area_compensated/'
    if not os.path.exists(dir):
        os.makedirs(dir)
    #for all csv files in the directory
    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        splitted = filename.split('_')
        map = splitted[0]
        if splitted[1] == 'tilted':
            map += '_tilted'
        spawn_time = splitted[-3]
        noise = splitted[-1].split('.')[0]

        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        time = data['time']
        
        agents = sorted(categories_and_values[map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        fig_all,ax_all = plt.subplots(len(agents),1)
        fig_free,ax_free = plt.subplots(len(agents),1)
        fig_occupied,ax_occupied = plt.subplots(len(agents),1)

        max_all = 0
        min_all = 1
        max_free = 0
        min_free = 1
        max_occupied = 0
        min_occupied = 1

        
        # color = 'tab:red'
        n_colors = int(((len(columns)-2)/3)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", (len(columns)-2)/3, '/', len(agents), '=', n_colors)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        i_irrelevant = 0
        for i,column in enumerate(columns):
            thisax = ax_all
            if column == 'time' or column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
            if column.endswith('free'):
                thisax = ax_free
                data[column] -= 0.5
                max_free = max(max_free, data[column].max())
                min_free = min(min_free, data[column].min())
            elif column.endswith('occupied'):
                thisax = ax_occupied
                data[column] = 0.5-data[column]
                max_occupied = max(max_occupied, data[column].max())
                min_occupied = min(min_occupied, data[column].min())
            elif column.endswith('all'):
                max_all = max(max_all, data[column].max())
                min_all = min(min_all, data[column].min())

            if 'std' in column:
                continue

            n_agents = column.split('_')[-3]
            agents_string = n_agents + '_agents'
            
            config = '_'.join(column.split('_')[0:6])

            #make sure the color is the same for all three certainty types
            #also make sure same config has same color, between the different agents
            color_index = hash(config) % n_colors
            color = colors[color_index]
            #label is column without agent number
            lbl = column.split('_')[:-3]
            lbl = '_'.join(lbl)
            thisax[agents.index(agents_string)].plot(time.to_numpy(), data[column].to_numpy(), label=lbl, color=color)
            if f"{column}_std" in data.columns:
                std_dev = data[f"{column}_std"].to_numpy()
                thisax[agents.index(agents_string)].fill_between(
                    time.to_numpy(),
                    data[column].to_numpy() - std_dev,
                    data[column].to_numpy() + std_dev,
                    color=color,
                    alpha=0.2
                )

        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            for thisax in [ax_all, ax_free, ax_occupied]:
                for t in data['time']:
                    thisax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
                thisax[agents.index(n_agents_string)].set_xlabel('')
                thisax[agents.index(n_agents_string)].set_ylabel('Certainty')
                thisax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
                thisax[agents.index(n_agents_string)].set_xlim(0, 400)
                # thisax[agents.index(n_agents_string)].set_ylim(0.05, 0.25)

        fig_all.tight_layout()
        fig_all.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        fig_all.suptitle(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, all certainty', wrap=True)
        ax_all[0].set_ylim(min_all, max_all)
        # fig_all.legend(loc='center left', fontsize=8)
        
        fig_free.tight_layout()
        fig_free.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        fig_free.suptitle(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, free certainty', wrap=True)
        ax_free[0].set_ylim(min_free, max_free)
        # fig_free.legend(loc='center left', fontsize=8)
        
        fig_occupied.tight_layout()
        fig_occupied.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        fig_occupied.suptitle(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, occupied certainty', wrap=True)
        ax_occupied[0].set_ylim(min_occupied, max_occupied)
        # fig_occupied.legend(loc='center left', fontsize=8)
        for a,f in [(ax_all, fig_all), (ax_free, fig_free), (ax_occupied, fig_occupied)]:
            #only keep unique labels in legend
            handles, labels = a[0].get_legend_handles_labels()
            unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
            f.legend(*zip(*unique), loc='upper right', fontsize=8)
        
        # plt.show()

def plot_certainty_with_different_configs_agents_line(plot_map, plot_noise, plot_spawn_time, usb_drive, categories_and_values, store_path):
    cell_types = 'all'

    dir = f'{usb_drive}averaged_data/{batch}/certainty_area_compensated/'
    if not os.path.exists(dir):
        os.makedirs(dir)
    alg = usb_drive.split('/')[-2]

    # Iterate through all CSV files in the directory
    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        splitted = filename.split('_')
        env_map = splitted[0]
        if splitted[1] == 'tilted':
            env_map += '_tilted'
        if env_map != plot_map:
            continue
        spawn_time = int(splitted[-3])
        if spawn_time != plot_spawn_time:
            continue
        noise = splitted[-1].replace('.csv', '')
        noise = float(noise)
        if plot_noise != noise:
            continue

        print("noise:", noise)
        print("spawn_time:", spawn_time)

        filepath = os.path.join(dir, filename)
        print("opening file:", filepath)
        data = pd.read_csv(filepath)
        # Prepend a row of 0
        data = pd.concat([pd.DataFrame([[0] + [0] * (len(data.columns) - 1)], columns=data.columns), data], ignore_index=True)
        columns = data.columns
        time = data['time']

        fig, ax = plt.subplots(figsize=(10, 6))

        # Generate colors for each agent group
        n_agents_columns = [col for col in columns if '_agents' in col and '_std' not in col and cell_types in col]
        print("n_agents_columns:", n_agents_columns)
        n_colors = len(n_agents_columns)
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in range(n_colors)]
        colors = [lighten_color(c, lighten_factor) for c in colors]

        for i, column in enumerate(n_agents_columns):
            if cell_types not in column:
                continue
            n_agents = column.split('_')[0]
            color = colors[i]
            certainty_data = data[column].to_numpy()
            ax.plot(time.to_numpy(), certainty_data, label=f"{n_agents} agents", color=color)
            if f"{column}_std" in data.columns:
                std_dev = data[f"{column}_std"].to_numpy()
                ax.fill_between(
                    time.to_numpy(),
                    certainty_data - std_dev,
                    certainty_data + std_dev,
                    color=color,
                    alpha=0.2
                )
            # Plot the average certainty line
            avg_certainty = np.mean(certainty_data)
            ax.axhline(y=avg_certainty, color=color, linestyle='--', alpha=0.7, label=f"{n_agents} agents AAC")

        ax.set_xlabel('Time (s)', fontsize=18)
        ax.set_ylabel('Certainty', fontsize=18)
        ax.set_xlim(0, end_time_for_map(env_map))
        ax.tick_params(axis='x', labelsize=16)
        ax.tick_params(axis='y', labelsize=16)
        handles, labels = ax.get_legend_handles_labels()
        filtered_handles_labels = [(h, l) for h, l in zip(handles, labels) if "AAC" not in l]
        filtered_handles, filtered_labels = zip(*filtered_handles_labels)
        ax.legend(filtered_handles, filtered_labels, fontsize=16)

        plt.tight_layout()
        #export figure
        if not os.path.exists(f'{store_path}results_{alg}/certainty_plots/over_time/{batch}/'):
            os.makedirs(f'{store_path}results_{alg}/certainty_plots/over_time/{batch}/')
        plt.savefig(f'{store_path}results_{alg}/certainty_plots/over_time/{batch}/{alg}_certainty_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')


def end_time_for_map(map):
    if 'house' in map:
        return 400
    elif 'office' in map:
        return 600
    elif 'museum' in map:
        return 1000
    
                   
def plot_coverage_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/coverage/'
    if not os.path.exists(dir):
        os.makedirs(dir)
    #for all csv files in the directory
    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        splitted = filename.split('_')
        env_map = splitted[0]
        if splitted[1] == 'tilted':
            env_map += '_tilted'
        spawn_time = int(splitted[-3])
        noise = splitted[-1].split('.')[0]
        #if splitted[-2] is a number, prepend it to noise with a decimal
        try:
            int(splitted[-2])
            noise = splitted[-2] + '.' + noise
        except:
            pass
        noise = float(noise)

        print("noise:", noise)
        print("spawn_time:", spawn_time)
        if noise != 0.0:
            continue
        if spawn_time != 0:
            continue

        #Set area size
        if env_map == 'house':
            # if spawn_time == 0:
                total_area=75.299072
        elif env_map == 'house_tilted':
            # if spawn_time == 0:
                total_area=78.604805
        elif env_map == 'office':
            # if spawn_time == 0:
                total_area=173.153870
        elif env_map == 'office_tilted':
            # if spawn_time == 0:
                total_area=176.347188
        elif env_map == 'museum':
            # if spawn_time == 0:
                total_area=689.311035
        elif env_map == 'museum_tilted':
            # if spawn_time == 0:
                total_area=701.826904

        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        #prepend a row of 0
        data = pd.concat([pd.DataFrame([[0] + [0]*(len(data.columns)-1)], columns=data.columns), data], ignore_index=True)
        columns = data.columns
        time = data['time']
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_colors = int((len(columns)-2)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-2, '/', len(agents), n_colors)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_colors)
        i_irrelevant = 0
        
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        average_coverages = {}
        for i,column in enumerate(columns):
            if column == 'time' or column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
            data[column] /= total_area
            data[column] *= 100            

            if 'std' in column:
                continue
            
            n_agents = column.split('_')[-2]
            # print("colorindex: ", (i-i_irrelevant)%n_colors)
            config = '_'.join(column.split('_')[0:6])

            #make sure the color is the same for all three certainty types
            #also make sure same config has same color, between the different agents
            # color_index = int(hashlib.md5(str(config).encode()).hexdigest(), 16) % n_colors
            #make sure the color is the same for all three certainty types
            #also make sure same config has same color, between the different agents
            if 'config_to_color_index' not in locals():
                config_to_color_index = {}
                current_color_index = 0

            if config not in config_to_color_index:
                config_to_color_index[config] = current_color_index
                current_color_index += 1
            
            color_index = config_to_color_index[config]

            color = colors[color_index]
            # color = colors[(i-i_irrelevant)%n_colors]   
            #label is column without agent number
            lbl = column.split('_')[:-2]
            lbl = '_'.join(lbl) 

            # fsr_symbol = '$R_f$'
            # fsr = lbl.split('_')[1] #R_f
            # if int(fsr) == 99999:
            #     fsr = '$\infty$'
            # mfr_symbol = '$N_f$'
            # mfr = lbl.split('_')[3] #N_f
            # if int(mfr) == 99999:
            #     mfr = '$\infty$'
            # mrl_symbol = '$N_s$'
            # mrl = lbl.split('_')[5] #N_s
            # if int(mrl) == 99999:
            #     mrl = '$\infty$'
            # lbl = f'{fsr_symbol}={fsr}, {mfr_symbol}={mfr}, {mrl_symbol}={mrl}'
            agents_string = n_agents + '_agents'
            ax[agents.index(agents_string)].plot(time.to_numpy(), data[column].to_numpy(), label=lbl, color=color)
            if f"{column}_std" in data.columns:
                std_dev = data[f"{column}_std"].to_numpy()
                ax[agents.index(agents_string)].fill_between(
                    time.to_numpy(),
                    data[column].to_numpy() - std_dev,
                    data[column].to_numpy() + std_dev,
                    color=color,
                    alpha=0.2
                )

            #get average coverage over time
            average_coverage = data[column].mean()
            if agents_string not in average_coverages:
                average_coverages[agents_string] = {}
            average_coverages[agents_string][lbl] = average_coverage

        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            ax[agents.index(n_agents_string)].set_xlabel('')
            ax[agents.index(n_agents_string)].set_ylabel('')
            ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            ax[agents.index(n_agents_string)].set_xlim(0, end_time_for_map(env_map))
            ax[agents.index(n_agents_string)].set_ylim(0,100)
            #data at end_time
            data_end_time = data.iloc[-1,2:]

            max_coverage = 0
            max_coverage_column = ''
            min_coverage = 999
            min_coverage_column = ''
            for i,column in enumerate(columns):
                if column == 'time' or column.startswith('Unnamed'):
                    continue
                #if column agents is not the same as n_agents_string, skip
                if column.split('_')[-2] != n_agents:
                    continue
                config = '_'.join(column.split('_')[0:6])
                lbl = column.split('_')[:-2]
                lbl = '_'.join(lbl) 

                # fsr_symbol = '$R_f$'
                # fsr = lbl.split('_')[1] #R_f
                # if int(fsr) == 99999:
                #     fsr = '$\infty$'
                # mfr_symbol = '$N_f$'
                # mfr = lbl.split('_')[3] #N_f
                # if int(mfr) == 99999:
                #     mfr = '$\infty$'
                # mrl_symbol = '$N_s$'
                # mrl = lbl.split('_')[5] #N_s
                # if int(mrl) == 99999:
                #     mrl = '$\infty$'
                # lbl = f'{fsr_symbol}={fsr}, {mfr_symbol}={mfr}, {mrl_symbol}={mrl}'
                coverage = data_end_time[column]
                if coverage > max_coverage:
                    max_coverage = coverage
                    max_coverage_column = lbl
                if coverage < min_coverage:
                    min_coverage = coverage
                    min_coverage_column = lbl
            end_time = end_time_for_map(env_map)
            ax[agents.index(n_agents_string)].text(end_time*0.625, 10, f'max CP({end_time}): {max_coverage_column}: {max_coverage:.2f}%', ha='left', va='center')
            ax[agents.index(n_agents_string)].text(end_time*0.625, 5, f'min CP({end_time}): {min_coverage_column}: {min_coverage:.2f}%', ha='left', va='center')
            
            max_average_coverage = 0
            max_average_coverage_column = ''
            min_average_coverage = 999
            min_average_coverage_column = ''
            for a in average_coverages[n_agents_string]:
                average_coverage = average_coverages[n_agents_string][a]
                if average_coverage > max_average_coverage:
                    max_average_coverage = average_coverage
                    max_average_coverage_column = a
                if average_coverage < min_average_coverage:
                    min_average_coverage = average_coverage
                    min_average_coverage_column = a
            ax[agents.index(n_agents_string)].text(end_time*0.250, 10, f'max ACP: {max_average_coverage_column}: {max_average_coverage:.2f}%', ha='left', va='center')
            ax[agents.index(n_agents_string)].text(end_time*0.250, 5, f'min ACP: {min_average_coverage_column}: {min_average_coverage:.2f}%', ha='left', va='center')
            

            # if 'max' not in counts['fsr']:
            #     counts['fsr']['max'] = {}
            # if 'min' not in counts['fsr']:
            #     counts['fsr']['min'] = {}
    
            # if 'max' not in counts['mfr']:
            #     counts['mfr']['max'] = {}
            # if 'min' not in counts['mfr']:
            #     counts['mfr']['min'] = {}
           
            # if 'max' not in counts['mrl']:
            #     counts['mrl']['max'] = {}
            # if 'min' not in counts['mrl']:
            #     counts['mrl']['min'] = {}

        

            # #get the fsr, mfr and mrl from the label
            # fsr_min = min_average_coverage_column.split('=')[1].split(',')[0]
            # mfr_min = min_average_coverage_column.split('=')[2].split(',')[0]
            # mrl_min = min_average_coverage_column.split('=')[3].split(',')[0]
            # fsr_max = max_average_coverage_column.split('=')[1].split(',')[0]
            # mfr_max = max_average_coverage_column.split('=')[2].split(',')[0]
            # mrl_max = max_average_coverage_column.split('=')[3].split(',')[0]

            
            # if fsr_max not in counts['fsr']['max']:
            #     counts['fsr']['max'][fsr_max] = 1
            # else:
            #     counts['fsr']['max'][fsr_max] += 1
            # if fsr_min not in counts['fsr']['min']:
            #     counts['fsr']['min'][fsr_min] = 1
            # else:
            #     counts['fsr']['min'][fsr_min] += 1

            # if mfr_max not in counts['mfr']['max']:
            #     counts['mfr']['max'][mfr_max] = 1
            # else:
            #     counts['mfr']['max'][mfr_max] += 1
            # if mfr_min not in counts['mfr']['min']:
            #     counts['mfr']['min'][mfr_min] = 1
            # else:
            #     counts['mfr']['min'][mfr_min] += 1

            # if mrl_max not in counts['mrl']['max']:
            #     counts['mrl']['max'][mrl_max] = 1
            # else:
            #     counts['mrl']['max'][mrl_max] += 1
            # if mrl_min not in counts['mrl']['min']:
            #     counts['mrl']['min'][mrl_min] = 1
            # else:
            #     counts['mrl']['min'][mrl_min] += 1
            


            #set legend
            # ax[agents.index(n_agents_string)].legend(loc='center right', fontsize=8)

        fig.text(0.04, 0.5, 'CP', va='center', rotation='vertical')
        #Add x-axis label
        fig.text(0.525, 0.02, 'Time (s)', ha='center')

        # fig.tight_layout()
        #set left, bottom, right, top, wspace, hspace
        # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        #only keep unique labels in legend
        handles, labels = ax[0].get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.9, 0.1), fontsize=8)
        
        plt.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        #set plot size
        fig.set_size_inches(14, 25)
        plt.show()
        # plt.savefig(f'coverage_plots/{batch}/coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'coverage_plots/{batch}/coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png')
        # #export counts to txt file, table format
        # with open(f'coverage_plots/{batch}/coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.txt', 'w') as f:
        #     f.write(f'Counts for map {env_map}, noise {noise}, spawn time {spawn_time}\n')
        #     f.write('Counts for max ACP\n')
        #     f.write(tabulate(counts['fsr']['max'].items(), headers=['FSR', 'Count'], tablefmt='grid'))
        #     f.write('\n')
        #     f.write(tabulate(counts['mfr']['max'].items(), headers=['MFR', 'Count'], tablefmt='grid'))
        #     f.write('\n')
        #     f.write(tabulate(counts['mrl']['max'].items(), headers=['MRL', 'Count'], tablefmt='grid'))
        #     f.write('\n')
        #     f.write('Counts for min ACP\n')
        #     f.write(tabulate(counts['fsr']['min'].items(), headers=['FSR', 'Count'], tablefmt='grid'))
        #     f.write('\n')
        #     f.write(tabulate(counts['mfr']['min'].items(), headers=['MFR', 'Count'], tablefmt='grid'))
        #     f.write('\n')
        #     f.write(tabulate(counts['mrl']['min'].items(), headers=['MRL', 'Count'], tablefmt='grid'))
        
def plot_coverage_with_different_configs_agents_line(plot_map, plot_noise, plot_spawn_time, usb_drive, categories_and_values, store_path):
    dir = f'{usb_drive}averaged_data/{batch}/coverage/'
    if not os.path.exists(dir):
        os.makedirs(dir)
    alg = usb_drive.split('/')[-2]
    #for all csv files in the directory
    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        splitted = filename.split('_')
        env_map = splitted[0]
        if splitted[1] == 'tilted':
            env_map += '_tilted'
        if env_map != plot_map:
            continue
        spawn_time = int(splitted[-3])
        if spawn_time != plot_spawn_time:
            continue
        noise = splitted[-1].replace('.csv', '')
        noise = float(noise)
        if plot_noise != noise:
            continue

        print("noise:", noise)
        print("spawn_time:", spawn_time)

        #Set area size
        if env_map == 'house':
            # if spawn_time == 0:
                total_area=75.299072
        elif env_map == 'house_tilted':
            # if spawn_time == 0:
                total_area=78.604805
        elif env_map == 'office':
            # if spawn_time == 0:
                total_area=173.153870
        elif env_map == 'office_tilted':
            # if spawn_time == 0:
                total_area=176.347188
        elif env_map == 'museum':
            # if spawn_time == 0:
                total_area=689.311035
        elif env_map == 'museum_tilted':
            # if spawn_time == 0:
                total_area=701.826904

        filepath = os.path.join(dir, filename)
    
        print("opening file:", filepath)
        data = pd.read_csv(filepath)
        # Prepend a row of 0
        data = pd.concat([pd.DataFrame([[0] + [0] * (len(data.columns) - 1)], columns=data.columns), data], ignore_index=True)
        columns = data.columns
        time = data['time']

        fig, ax = plt.subplots(figsize=(10, 6))

        # Generate colors for each agent group
        n_agents_columns = [col for col in columns if '_agents' in col and '_std' not in col]
        n_colors = len(n_agents_columns)
        # colors = generate_high_contrast_hsv_colors(n_colors)
        #tab10 colors
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in range(n_colors)]    
        colors = [lighten_color(c, lighten_factor) for c in colors]    

        for i, column in enumerate(n_agents_columns):
            n_agents = column.split('_')[0]
            color = colors[i]
            coverage_data = (data[column].to_numpy() / total_area) * 100
            ax.plot(time.to_numpy(), coverage_data, label=f"{n_agents} agents", color=color)
            if f"{column}_std" in data.columns:
                std_dev = (data[f"{column}_std"].to_numpy() / total_area) * 100
                data_minus_std = (data[column].to_numpy() - std_dev) / total_area * 100
                data_plus_std = (data[column].to_numpy() + std_dev) / total_area * 100
                ax.fill_between(
                    time.to_numpy(),
                    data_minus_std,
                    data_plus_std,
                    color=color,
                    alpha=0.2
                )
            # Plot the average coverage line
            avg_coverage = np.mean(coverage_data)
            ax.axhline(y=avg_coverage, color=color, linestyle='--', alpha=0.7, label=f"{n_agents} agents ACP")
        ax.set_xlabel('Time (s)', fontsize=18)
        ax.set_ylabel('CP (%)', fontsize=18)
        # ax.set_title(f'Coverage for map {env_map}, noise {noise}, spawn time {spawn_time}')
        ax.set_xlim(0, end_time_for_map(env_map))
        #set xticks fontsize
        ax.tick_params(axis='x', labelsize=16)
        ax.set_ylim(0, 100)
        #set yticks fontsize
        ax.tick_params(axis='y', labelsize=16)
        handles, labels = ax.get_legend_handles_labels()
        filtered_handles_labels = [(h, l) for h, l in zip(handles, labels) if "ACP" not in l]
        filtered_handles, filtered_labels = zip(*filtered_handles_labels)
        ax.legend(filtered_handles, filtered_labels, loc='lower right', fontsize=16)

        plt.tight_layout()
        #export figure
        if not os.path.exists(f'{store_path}results_{alg}/coverage_plots/over_time/{batch}'):
            os.makedirs(f'{store_path}results_{alg}/coverage_plots/over_time/{batch}')
        plt.savefig(f'{store_path}results_{alg}/coverage_plots/over_time/{batch}/{alg}_coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')

        # plt.show()
    # for i, column in enumerate(n_agents_columns):
    #     n_agents = column.split('_')[0]
    #     color = colors[i]
    #     coverage_data = (data[column].to_numpy() / total_area) * 100
    #     ax.plot(time.to_numpy(), coverage_data, label=f"{n_agents} agents", color=color)
    #     if f"{column}_std" in data.columns:
    #         std_dev = (data[f"{column}_std"].to_numpy() / total_area) * 100
    #         data_minus_std = (data[column].to_numpy() - std_dev) / total_area * 100
    #         data_plus_std = (data[column].to_numpy() + std_dev) / total_area * 100
    #         ax.fill_between(
    #             time.to_numpy(),
    #             data_minus_std,
    #             data_plus_std,
    #             color=color,
    #             alpha=0.2
    #         )

def generate_average_certainty_csv_files():
    """
    Generate CSV files with average certainty data for each map, noise level, and spawn time combination.
    Each file will have columns for different algorithms.
    """
    maps = ['house', 'house_tilted', 'office', 'office_tilted', 'museum', 'museum_tilted']
    noises = [0, 1]
    spawn_times = [0]
    cell_types = ['all', 'free', 'occupied']
    # Only consider BICLARE-PP, BICLARE-WF and FSP-QT
    algorithms = [
        {'name': 'BICLARE-PP', 'path': '/media/hugo/Thesis_Data/CLARE/', 'batch': 'noise'},
        {'name': 'BICLARE-WF', 'path': '/media/hugo/Thesis_Data/CLARE_wallfollowing/', 'batch': 'noise_wallfollowing'},
        {'name': 'FSP-QT', 'path': '/media/hugo/Thesis_Data/Base_QT/', 'batch': 'noise_QT'}
    ]
    
    # Create output directory if it doesn't exist
    output_dir = 'results_comp/certainty/csv_comp'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Process each combination of map, noise, spawn time and cell type
    for env_map in maps:
        for noise in noises:
            for spawn_time in spawn_times:
                for cell_type in cell_types:
                    # Dictionary to store data for each algorithm
                    all_data = {}
                    
                    # Collect data from each algorithm
                    for algo in algorithms:
                        data_path = f"{algo['path']}averaged_data/{algo['batch']}/certainty_area_compensated/"
                        if not os.path.exists(data_path):
                            print(f"Path doesn't exist: {data_path}")
                            continue
                        
                        # Find the matching file
                        for file in os.listdir(data_path):
                            filename = os.fsdecode(file)
                            splitted = filename.split('_')
                            file_map = splitted[0]
                            if splitted[1] == 'tilted':
                                file_map += '_tilted'
                            if file_map != env_map:
                                continue
                            
                            file_spawn_time = int(splitted[-3])
                            if file_spawn_time != spawn_time:
                                continue
                            
                            file_noise = splitted[-1].replace('.csv', '')
                            try:
                                file_noise = float(file_noise)
                                if file_noise != noise:
                                    continue
                            except ValueError:
                                continue
                            
                            # Found matching file, extract data
                            filepath = os.path.join(data_path, filename)
                            print(f"Reading file: {filepath} for algorithm {algo['name']}")
                            
                            df = pd.read_csv(filepath)
                            # Prepend a row of 0
                            df = pd.concat([pd.DataFrame([[0] + [0] * (len(df.columns) - 1)], columns=df.columns), df], ignore_index=True)
                            
                            # Filter data to end time
                            df = df[df['time'] <= end_time_for_map(env_map)]
                            
                            for col in df.columns:
                                # Filter by cell type and exclude std columns
                                if f"_{cell_type}" in col and not col.endswith('_std') and col != 'time' and not col.startswith('Unnamed'):
                                    n_agents = col.split('_')[-3]  # Extract agent number
                                    agent_key = f"{n_agents}_agents"
                                    
                                    # Calculate average certainty
                                    certainty_value = df[col].mean()
                                    
                                    if agent_key not in all_data:
                                        all_data[agent_key] = {}
                                    
                                    all_data[agent_key][algo['name']] = certainty_value
                    
                    # Create and save CSV
                    if all_data:
                        output_file = f"{output_dir}/avg_certainty_{cell_type}_{env_map}_noise_{noise}_spawn_{spawn_time}.csv"
                        
                        # Convert to DataFrame
                        csv_data = []
                        for agent_key in sorted(all_data.keys(), key=lambda x: int(x.split('_')[0])):
                            row = {'agents': agent_key}
                            row.update(all_data[agent_key])
                            csv_data.append(row)
                        
                        pd.DataFrame(csv_data).to_csv(output_file, index=False)
                        print(f"Created: {output_file}")

def generate_average_coverage_csv_files():
    """
    Generate CSV files with average coverage data for each map, noise level, and spawn time combination.
    Each file will have columns for different algorithms.
    """
    maps = ['house', 'house_tilted', 'office', 'office_tilted', 'museum', 'museum_tilted']
    noises = [0, 1]
    spawn_times = [0]
    algorithms = [
        {'name': 'BICLARE-PP', 'path': '/media/hugo/Thesis_Data/CLARE/', 'batch': 'noise'},
        {'name': 'BICLARE-WF', 'path': '/media/hugo/Thesis_Data/CLARE_wallfollowing/', 'batch': 'noise_wallfollowing'},
        {'name': 'FSP-QT', 'path': '/media/hugo/Thesis_Data/Base_QT/', 'batch': 'noise_QT'},
        {'name': 'FSP-G', 'path': '/media/hugo/Thesis_Data/Base_cohesion_alignment/', 'batch': 'noise_cohesionalignment'},
        {'name': 'FSP', 'path': '/media/hugo/Thesis_Data/Base/', 'batch': 'noise'}
    ]
    
    # Create output directory if it doesn't exist
    output_dir = 'results_comp/coverage/csv_comp'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Process each combination of map, noise, and spawn time
    for env_map in maps:
        for noise in noises:
            for spawn_time in spawn_times:
                # Dictionary to store data for each algorithm
                all_data = {}
                total_area = get_total_area(env_map)
                
                # Collect data from each algorithm
                for algo in algorithms:
                    data_path = f"{algo['path']}averaged_data/{algo['batch']}/coverage/"
                    if not os.path.exists(data_path):
                        print(f"Path doesn't exist: {data_path}")
                        continue
                    
                    # Find the matching file
                    for file in os.listdir(data_path):
                        filename = os.fsdecode(file)
                        splitted = filename.split('_')
                        file_map = splitted[0]
                        if splitted[1] == 'tilted':
                            file_map += '_tilted'
                        if file_map != env_map:
                            continue
                        
                        file_spawn_time = int(splitted[-3])
                        if file_spawn_time != spawn_time:
                            continue
                        
                        file_noise = splitted[-1].replace('.csv', '')
                        file_noise = float(file_noise)
                        if file_noise != noise:
                            continue
                        
                        # Found matching file, extract data
                        filepath = os.path.join(data_path, filename)
                        print(f"Reading file: {filepath} for algorithm {algo['name']}")
                        
                        df = pd.read_csv(filepath)
                        # Prepend a row of 0
                        df = pd.concat([pd.DataFrame([[0] + [0] * (len(df.columns) - 1)], columns=df.columns), df], ignore_index=True)
                        # Convert raw coverage to percentage
                        for col in df.columns:
                            if col != 'time' and not col.startswith('Unnamed') and not col.endswith('_std'):
                                n_agents = col.split('_')[0]
                                agent_key = f"{n_agents}_agents"

                                #only include up to end_time
                                df = df[df['time'] <= end_time_for_map(env_map)]
                                
                                coverage_data = (df[col].to_numpy() / total_area) * 100
                                coverage_percentage = coverage_data.mean()
                                
                                # Calculate average coverage percentage
                                # coverage_percentage = (df[col].mean() / total_area) * 100
                                
                                if agent_key not in all_data:
                                    all_data[agent_key] = {}
                                
                                all_data[agent_key][algo['name']] = coverage_percentage
                
                # Create and save CSV
                if all_data:
                    output_file = f"{output_dir}/avg_coverage_{env_map}_noise_{noise}_spawn_{spawn_time}.csv"
                    
                    # Convert to DataFrame
                    csv_data = []
                    for agent_key in sorted(all_data.keys(), key=lambda x: int(x.split('_')[0])):
                        row = {'agents': agent_key}
                        row.update(all_data[agent_key])
                        csv_data.append(row)
                    
                    pd.DataFrame(csv_data).to_csv(output_file, index=False)
                    print(f"Created: {output_file}")

def get_total_area(env_map):
    """Return the total area for a given map."""
    area_mapping = {
        'house': 75.299072,
        'house_tilted': 78.604805,
        'office': 173.153870,
        'office_tilted': 176.347188,
        'museum': 689.311035,
        'museum_tilted': 701.826904
    }
    return area_mapping.get(env_map, 0)

def combine_csv_files(source_dir, output_file, data_type):
    """
    Combine multiple CSV files into a single large CSV file.
    
    Args:
        source_dir (str): Directory containing the CSV files to combine
        output_file (str): Path for the output combined CSV file
        data_type (str): Type of data ('coverage' or 'certainty')
    """
    if not os.path.exists(source_dir):
        print(f"Source directory {source_dir} does not exist")
        return
    
    # Create empty DataFrame to hold all data
    combined_df = None
    
    # Process all CSV files in the directory
    for file in os.listdir(source_dir):
        if not file.endswith('.csv') or file.startswith('combined_'):
            continue
            
        filepath = os.path.join(source_dir, file)
        df = pd.read_csv(filepath)
        
        # Extract metadata from filename
        filename_parts = file.split('_')
        
        # For certainty files: avg_certainty_{cell_type}_{env_map}_noise_{noise}_spawn_{spawn_time}.csv
        # For coverage files: avg_coverage_{env_map}_noise_{noise}_spawn_{spawn_time}.csv
        
        if data_type == 'certainty':
            cell_type = filename_parts[2]
            #only include cell_type 'all'
            if cell_type != 'all':
                continue
            if 'tilted' in file:
                env_map = f"{filename_parts[3]}_{filename_parts[4]}"
                noise_idx = 6
            else:
                env_map = filename_parts[3]
                noise_idx = 5
            noise = filename_parts[noise_idx].split('.')[0]
        else:  # coverage
            if 'tilted' in file:
                env_map = f"{filename_parts[2]}_{filename_parts[3]}"
                noise_idx = 5
            else:
                env_map = filename_parts[2]
                noise_idx = 4
            noise = filename_parts[noise_idx].split('.')[0]
            cell_type = "N/A"  # Not applicable for coverage
            
        # Add metadata columns
        df['map'] = env_map
        df['noise'] = noise
        
        # if data_type == 'certainty':
        #     df['cell_type'] = cell_type
            
        # Add to combined DataFrame
        if combined_df is None:
            combined_df = df
        else:
            combined_df = pd.concat([combined_df, df], ignore_index=True)
    
    if combined_df is not None:
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        # Save combined data
        combined_df.to_csv(output_file, index=False)
        print(f"Combined data saved to {output_file}")
    else:
        print("No CSV files were found or processed")

def combine_all_comparison_data():
    """Combine all comparison CSV files for both coverage and certainty"""
    # Combine coverage data
    coverage_source = 'results_comp/coverage/csv_comp'
    coverage_output = 'results_comp/coverage/csv_comp/combined_coverage_data.csv'
    combine_csv_files(coverage_source, coverage_output, 'coverage')
    
    # Combine certainty data
    certainty_source = 'results_comp/certainty/csv_comp'
    certainty_output = 'results_comp/certainty/csv_comp/combined_certainty_data.csv'
    combine_csv_files(certainty_source, certainty_output, 'certainty')
       
# Function to lighten an RGBA color
def lighten_color(color, amount=0.5):
    """
    Lightens the given color by mixing it with white.
    amount: 0 < amount <= 1, where 1 is fully white.
    """
    import matplotlib.colors as mc
    c = mc.to_rgb(color)
    c_white = np.array([1.0, 1.0, 1.0])
    return mc.to_rgba(c + (c_white - c) * amount)

lighten_factor = 0.2

usb_drive_dirs = ['/media/hugo/Thesis_Data/CLARE/', '/media/hugo/Thesis_Data/CLARE_wallfollowing/', '/media/hugo/Thesis_Data/Base_QT/', '/media/hugo/Thesis_Data/Base_cohesion_alignment/', '/media/hugo/Thesis_Data/Base/']
batches = ['noise', 'noise_wallfollowing', 'noise_QT', 'noise_cohesionalignment', 'noise']
config_paths = ['implementation_and_examples/agent_implementation/configs/', 'implementation_and_examples/agent_implementation/configs/', '../Thesis_ARGoS2/implementation_and_examples/agent_implementation/configs/', '../Thesis_ARGoS2/implementation_and_examples/agent_implementation/configs/', '../Thesis_ARGoS2/implementation_and_examples/agent_implementation/configs/']
store_paths = ['', '', '../Thesis_ARGoS2/', '../Thesis_ARGoS2/', '../Thesis_ARGoS2/']
for i, usb_drive in enumerate(usb_drive_dirs):
    batch = batches[i]
    store_path = store_paths[i]


    configs = []

    #get all files in 'implementation_and_examples/agent_implementation/configs/{batch}'
    for file in os.listdir(f'{config_paths[i]}{batch}'):
        config_name = file.split('.')[0]
        configs.append(config_name)    

    # zipfiles = []
    # completed_per_zip = []
    # # for all files in the usb drive
    # for usb_drive in usb_drive_dirs:
    #     for file in os.listdir(usb_drive):
    #         if file.endswith(".zip"):
    #             if file =='experiment_results_afternoon_11-03.zip':
    #                 continue
    #             zip_file = usb_drive + file
    #             zipfiles.append(zip_file)

    # #if directory  does not exist, create it
    # if not os.path.exists('coverage_plots'):
    #     os.makedirs('coverage_plots')

    # #if directory  does not exist, create it
    # if not os.path.exists('certainty_plots'):
    #     os.makedirs('certainty_plots')



    completed_experiments, categories_and_values = get_values_for_each_category(configs)   
    # check_if_all_required_experiments_done(completed_experiments, categories_and_values)
        # plot_certainty_with_different_configs(usb_drive, categories_and_values)
        # plot_coverage_with_different_configs(usb_drive, categories_and_values)
        # for env_map in ['house', 'office', 'museum']:
    for env_map in ['house' ,'house_tilted', 'office', 'office_tilted', 'museum', 'museum_tilted']:
        for noise in [0, 1]:
            plot_coverage_with_different_configs_agents_line(env_map, noise, 0, usb_drive, categories_and_values, store_path)
            plt.close()
            plot_certainty_with_different_configs_agents_line(env_map, noise, 0, usb_drive, categories_and_values, store_path)
            plt.close()

# Call the function to generate CSV files
generate_average_coverage_csv_files()
generate_average_certainty_csv_files()
combine_all_comparison_data()
# plt.show()