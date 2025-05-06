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
from statistics import mean

ticks_per_second = 16
prefix = ''

batch = "noise"

algorithm_dir = 'CLARE'

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

                   
def plot_traveled_path_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/traveled_path/'
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        average_traveled_paths = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue

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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            average_traveled_path = data[column].mean() #should be only 1 value
            if agents_string not in average_traveled_paths:
                average_traveled_paths[agents_string] = {}
            average_traveled_paths[agents_string][lbl] = average_traveled_path

        average_traveled_path_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)


            max_traveled_path = 0
            max_traveled_path_column = ''
            min_traveled_path = 999
            min_traveled_path_column = ''
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
                traveled_path = data[column].to_numpy()[-1]
                if traveled_path > max_traveled_path:
                    max_traveled_path = traveled_path
                    max_traveled_path_column = config
                if traveled_path < min_traveled_path:
                    min_traveled_path = traveled_path
                    min_traveled_path_column = config

            
            min_traveled_path_color_index = config_to_color_index[min_traveled_path_column]
            max_traveled_path_color_index = config_to_color_index[max_traveled_path_column]
            # ax[agents.index(n_agents_string)].set_ylim(total_min * 0.9, total_max * 1.1)
            # ax[agents.index(n_agents_string)].bar(min_traveled_path_color_index, min_traveled_path, facecolor='none', edgecolor='springgreen', linewidth=4)
            # ax[agents.index(n_agents_string)].bar(max_traveled_path_color_index, max_traveled_path, facecolor='none', edgecolor='darkslategray', linewidth=4)

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

        

            #get the fsr, mfr and mrl from the label
            # fsr_min = min_traveled_path_column.split('_')[1] #fsr
            # mfr_min = min_traveled_path_column.split('_')[3] #mfr
            # mrl_min = min_traveled_path_column.split('_')[5] #mrl
            # fsr_max = max_traveled_path_column.split('_')[1] #fsr
            # mfr_max = max_traveled_path_column.split('_')[3] #mfr
            # mrl_max = max_traveled_path_column.split('_')[5] #mrl

            
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

            traveled_path_stats = {}
            traveled_path_for_n_agents = list(average_traveled_paths[n_agents_string].values())
            # traveled_path_stats['min'] = min(traveled_path_for_n_agents)
            # traveled_path_stats['median'] = np.median(traveled_path_for_n_agents)
            # traveled_path_stats['max'] = max(traveled_path_for_n_agents)
            traveled_path_stats['mean'] = mean(traveled_path_for_n_agents)
            traveled_path_stats['Standard Deviation'] = np.std(traveled_path_for_n_agents)

            average_traveled_path_stats_per_n_agents[n_agents_string] = traveled_path_stats


        traveled_path_for_n_agents_df = pd.DataFrame.from_dict(average_traveled_path_stats_per_n_agents, orient='index').reset_index()
        traveled_path_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/traveled_path/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/traveled_path/{batch}/csv_tables')
        traveled_path_for_n_agents_df.to_csv(f'results_{algorithm_dir}/traveled_path/{batch}/csv_tables/traveled_path_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)


        # fig.text(0.04, 0.5, 'Traveled path (m)', va='center', rotation='vertical')
        # plt.xticks(x-0.5, config_labels, rotation=45)

        # fig.tight_layout()
        #set left, bottom, right, top, wspace, hspace
        # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        #only keep unique labels in legend
        # handles, labels = ax[0].get_legend_handles_labels()
        # unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.9, 0.1), fontsize=8)
        
        # plt.subplots_adjust(left=0.08, bottom=0.1, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        #set plot size
        # fig.set_size_inches(14, 25)
        # plt.show()
        # plt.savefig(f'traveled_path/{batch}/traveled_path_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'traveled_path/{batch}/traveled_path_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png')
        # #export counts to txt file, table format
        # with open(f'traveled_path/{batch}/traveled_path_{env_map}_noise_{noise}_spawn_time_{spawn_time}.txt', 'w') as f:
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

def plot_battery_usage_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/battery_usage/'
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        # counts = {}
        # counts['mrl'] = {}
        # counts['mfr'] = {}
        # counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        average_battery_usages = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            average_battery_usage = data[column].mean() #should be only 1 value
            if agents_string not in average_battery_usages:
                average_battery_usages[agents_string] = {}
            average_battery_usages[agents_string][lbl] = average_battery_usage

        average_battery_usage_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)


            max_battery = 0
            max_battery_column = ''
            min_battery = 999
            min_battery_column = ''
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
                battery = data[column].to_numpy()[-1]
                if battery > max_battery:
                    max_battery = battery
                    max_battery_column = config
                if battery < min_battery:
                    min_battery = battery
                    min_battery_column = config

            
            min_battery_color_index = config_to_color_index[min_battery_column]
            max_battery_color_index = config_to_color_index[max_battery_column]
            # ax[agents.index(n_agents_string)].set_ylim(total_min * 0.9, total_max * 1.1)
            # ax[agents.index(n_agents_string)].bar(min_battery_color_index, min_battery, facecolor='none', edgecolor='springgreen', linewidth=4)
            # ax[agents.index(n_agents_string)].bar(max_battery_color_index, max_battery, facecolor='none', edgecolor='darkslategray', linewidth=4)

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
            # fsr_min = min_battery_column.split('_')[1] #fsr
            # mfr_min = min_battery_column.split('_')[3] #mfr
            # mrl_min = min_battery_column.split('_')[5] #mrl
            # fsr_max = max_battery_column.split('_')[1] #fsr
            # mfr_max = max_battery_column.split('_')[3] #mfr
            # mrl_max = max_battery_column.split('_')[5] #mrl

            
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

            battery_usage_stats = {}
            battery_usage_for_n_agents = list(average_battery_usages[n_agents_string].values())
            # battery_usage_stats['min'] = min(battery_usage_for_n_agents)
            # battery_usage_stats['median'] = np.median(battery_usage_for_n_agents)
            # battery_usage_stats['max'] = max(battery_usage_for_n_agents)
            battery_usage_stats['mean'] = mean(battery_usage_for_n_agents)
            battery_usage_stats['Standard Deviation'] = np.std(battery_usage_for_n_agents)

            average_battery_usage_stats_per_n_agents[n_agents_string] = battery_usage_stats

        battery_usage_for_n_agents_df = pd.DataFrame.from_dict(average_battery_usage_stats_per_n_agents, orient='index').reset_index()
        battery_usage_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/battery_usage/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/battery_usage/{batch}/csv_tables')
        battery_usage_for_n_agents_df.to_csv(f'results_{algorithm_dir}/battery_usage/{batch}/csv_tables/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

        # fig.text(0.04, 0.5, 'Traveled path (m)', va='center', rotation='vertical')
        # plt.xticks(x-0.5, config_labels, rotation=45)

        # fig.tight_layout()
        #set left, bottom, right, top, wspace, hspace
        # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        #only keep unique labels in legend
        # handles, labels = ax[0].get_legend_handles_labels()
        # unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.9, 0.1), fontsize=8)
        
        # plt.subplots_adjust(left=0.08, bottom=0.1, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # #set plot size
        # fig.set_size_inches(14, 25)
        # # plt.show()
        # plt.savefig(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png')
        # #export counts to txt file, table format
        # with open(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.txt', 'w') as f:
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
    
def plot_bytes_sent_received_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/bytes_sent_received/'
    if not os.path.exists(dir):
        print(f'Directory {dir} does not exist')
        return
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        averages_bytes_sent = {}
        averages_bytes_received = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
            if 'std' in column:
                continue


            n_agents = column.split('_')[-4]
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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            if column.endswith('bytes_sent'):
                average_bytes_sent = data[column].mean() #should be only 1 value
                if agents_string not in averages_bytes_sent:
                    averages_bytes_sent[agents_string] = {}
                averages_bytes_sent[agents_string][lbl] = average_bytes_sent
            elif column.endswith('bytes_received'):
                average_bytes_received = data[column].mean()
                if agents_string not in averages_bytes_received:
                    averages_bytes_received[agents_string] = {}
                averages_bytes_received[agents_string][lbl] = average_bytes_received


        average_bytes_sent_received_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)


            # min_bytes_sent = 0
            # max_battery_column = ''
            # min_battery = 999
            # min_battery_column = ''
            # for i,column in enumerate(columns):
            #     if column == 'time' or column.startswith('Unnamed'):
            #         continue
            #     #if column agents is not the same as n_agents_string, skip
            #     if column.split('_')[-2] != n_agents:
            #         continue
            #     config = '_'.join(column.split('_')[0:6])
            #     lbl = column.split('_')[:-2]
            #     lbl = '_'.join(lbl) 

            #     fsr_symbol = '$R_f$'
            #     fsr = lbl.split('_')[1] #R_f
            #     if int(fsr) == 99999:
            #         fsr = '$\infty$'
            #     mfr_symbol = '$N_f$'
            #     mfr = lbl.split('_')[3] #N_f
            #     if int(mfr) == 99999:
            #         mfr = '$\infty$'
            #     mrl_symbol = '$N_s$'
            #     mrl = lbl.split('_')[5] #N_s
            #     if int(mrl) == 99999:
            #         mrl = '$\infty$'
            #     lbl = f'{fsr_symbol}={fsr}, {mfr_symbol}={mfr}, {mrl_symbol}={mrl}'
            #     battery = data[column].to_numpy()[-1]
            #     if battery > min_bytes_sent:
            #         min_bytes_sent = battery
            #         max_battery_column = config
            #     if battery < min_battery:
            #         min_battery = battery
            # #         min_battery_column = config

            
            # # min_battery_color_index = config_to_color_index[min_battery_column]
            # # max_battery_color_index = config_to_color_index[max_battery_column]
            # # ax[agents.index(n_agents_string)].set_ylim(total_min * 0.9, total_max * 1.1)
            # # ax[agents.index(n_agents_string)].bar(min_battery_color_index, min_battery, facecolor='none', edgecolor='springgreen', linewidth=4)
            # # ax[agents.index(n_agents_string)].bar(max_battery_color_index, max_battery, facecolor='none', edgecolor='darkslategray', linewidth=4)

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
            # fsr_min = min_battery_column.split('_')[1] #fsr
            # mfr_min = min_battery_column.split('_')[3] #mfr
            # mrl_min = min_battery_column.split('_')[5] #mrl
            # fsr_max = max_battery_column.split('_')[1] #fsr
            # mfr_max = max_battery_column.split('_')[3] #mfr
            # mrl_max = max_battery_column.split('_')[5] #mrl

            
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

            bytes_sent_received_stats = {}
            bytes_sent_for_n_agents = list(averages_bytes_sent[n_agents_string].values())
            bytes_received_for_n_agents = list(averages_bytes_received[n_agents_string].values())
            # bytes_sent_received_stats['min sent'] = min(bytes_sent_for_n_agents)
            # bytes_sent_received_stats['median sent'] = np.median(bytes_sent_for_n_agents)
            # bytes_sent_received_stats['max sent'] = max(bytes_sent_for_n_agents)
            bytes_sent_received_stats['mean sent'] = mean(bytes_sent_for_n_agents)
            bytes_sent_received_stats['Standard Deviation sent'] = np.std(bytes_sent_for_n_agents)
            # bytes_sent_received_stats['min received'] = min(bytes_received_for_n_agents)
            # bytes_sent_received_stats['median received'] = np.median(bytes_received_for_n_agents)
            # bytes_sent_received_stats['max received'] = max(bytes_received_for_n_agents)
            bytes_sent_received_stats['mean received'] = mean(bytes_received_for_n_agents)
            bytes_sent_received_stats['Standard Deviation received'] = np.std(bytes_received_for_n_agents)

            average_bytes_sent_received_stats_per_n_agents[n_agents_string] = bytes_sent_received_stats

        bytes_sent_received_for_n_agents_df = pd.DataFrame.from_dict(average_bytes_sent_received_stats_per_n_agents, orient='index').reset_index()
        bytes_sent_received_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/bytes_sent_received/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/bytes_sent_received/{batch}/csv_tables')
        bytes_sent_received_for_n_agents_df.to_csv(f'results_{algorithm_dir}/bytes_sent_received/{batch}/csv_tables/bytes_sent_received_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

        # fig.text(0.04, 0.5, 'Traveled path (m)', va='center', rotation='vertical')
        # plt.xticks(x-0.5, config_labels, rotation=45)

        # fig.tight_layout()
        #set left, bottom, right, top, wspace, hspace
        # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        #only keep unique labels in legend
        # handles, labels = ax[0].get_legend_handles_labels()
        # unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.9, 0.1), fontsize=8)
        
        # plt.subplots_adjust(left=0.08, bottom=0.1, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # #set plot size
        # fig.set_size_inches(14, 25)
        # # plt.show()
        # plt.savefig(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png')
        # #export counts to txt file, table format
        # with open(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.txt', 'w') as f:
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

def plot_max_nodes_leaves_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/max_nodes_and_leaves/'
    if not os.path.exists(dir):
        print(f'Directory {dir} does not exist')
        return
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        average_max_nodes = {}
        average_max_leaves = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
            if 'std' in column:
                continue


            n_agents = column.split('_')[-4]
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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            if column.endswith('max_leaves'):
                average_bytes_sent = data[column].mean() #should be only 1 value
                if agents_string not in average_max_nodes:
                    average_max_nodes[agents_string] = {}
                average_max_nodes[agents_string][lbl] = average_bytes_sent
            elif column.endswith('max_nodes'):
                average_bytes_received = data[column].mean()
                if agents_string not in average_max_leaves:
                    average_max_leaves[agents_string] = {}
                average_max_leaves[agents_string][lbl] = average_bytes_received


        average_max_nodes_and_leaves_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)


            # min_bytes_sent = 0
            # max_battery_column = ''
            # min_battery = 999
            # min_battery_column = ''
            # for i,column in enumerate(columns):
            #     if column == 'time' or column.startswith('Unnamed'):
            #         continue
            #     #if column agents is not the same as n_agents_string, skip
            #     if column.split('_')[-2] != n_agents:
            #         continue
            #     config = '_'.join(column.split('_')[0:6])
            #     lbl = column.split('_')[:-2]
            #     lbl = '_'.join(lbl) 

            #     fsr_symbol = '$R_f$'
            #     fsr = lbl.split('_')[1] #R_f
            #     if int(fsr) == 99999:
            #         fsr = '$\infty$'
            #     mfr_symbol = '$N_f$'
            #     mfr = lbl.split('_')[3] #N_f
            #     if int(mfr) == 99999:
            #         mfr = '$\infty$'
            #     mrl_symbol = '$N_s$'
            #     mrl = lbl.split('_')[5] #N_s
            #     if int(mrl) == 99999:
            #         mrl = '$\infty$'
            #     lbl = f'{fsr_symbol}={fsr}, {mfr_symbol}={mfr}, {mrl_symbol}={mrl}'
            #     battery = data[column].to_numpy()[-1]
            #     if battery > min_bytes_sent:
            #         min_bytes_sent = battery
            #         max_battery_column = config
            #     if battery < min_battery:
            #         min_battery = battery
            # #         min_battery_column = config

            
            # # min_battery_color_index = config_to_color_index[min_battery_column]
            # # max_battery_color_index = config_to_color_index[max_battery_column]
            # # ax[agents.index(n_agents_string)].set_ylim(total_min * 0.9, total_max * 1.1)
            # # ax[agents.index(n_agents_string)].bar(min_battery_color_index, min_battery, facecolor='none', edgecolor='springgreen', linewidth=4)
            # # ax[agents.index(n_agents_string)].bar(max_battery_color_index, max_battery, facecolor='none', edgecolor='darkslategray', linewidth=4)

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
            # fsr_min = min_battery_column.split('_')[1] #fsr
            # mfr_min = min_battery_column.split('_')[3] #mfr
            # mrl_min = min_battery_column.split('_')[5] #mrl
            # fsr_max = max_battery_column.split('_')[1] #fsr
            # mfr_max = max_battery_column.split('_')[3] #mfr
            # mrl_max = max_battery_column.split('_')[5] #mrl

            
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

            max_nodes_leaves_stats = {}
            max_nodes_for_n_agents = list(average_max_nodes[n_agents_string].values())
            max_leaves_for_n_agents = list(average_max_leaves[n_agents_string].values())
            # max_nodes_leaves_stats['min max nodes'] = min(max_nodes_for_n_agents)
            # max_nodes_leaves_stats['median max nodes'] = np.median(max_nodes_for_n_agents)
            # max_nodes_leaves_stats['max max nodes'] = max(max_nodes_for_n_agents)
            max_nodes_leaves_stats['mean max nodes'] = mean(max_nodes_for_n_agents)
            max_nodes_leaves_stats['Standard Deviation max nodes'] = np.std(max_nodes_for_n_agents)
            # max_nodes_leaves_stats['min max leaves'] = min(max_leaves_for_n_agents)
            # max_nodes_leaves_stats['median max leaves'] = np.median(max_leaves_for_n_agents)
            # max_nodes_leaves_stats['max max leaves'] = max(max_leaves_for_n_agents)
            max_nodes_leaves_stats['mean max leaves'] = mean(max_leaves_for_n_agents)
            max_nodes_leaves_stats['Standard Deviation max leaves'] = np.std(max_leaves_for_n_agents)

            average_max_nodes_and_leaves_stats_per_n_agents[n_agents_string] = max_nodes_leaves_stats

        max_nodes_leaves_for_n_agents_df = pd.DataFrame.from_dict(average_max_nodes_and_leaves_stats_per_n_agents, orient='index').reset_index()
        max_nodes_leaves_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/max_nodes_and_leaves/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/max_nodes_and_leaves/{batch}/csv_tables')
        max_nodes_leaves_for_n_agents_df.to_csv(f'results_{algorithm_dir}/max_nodes_and_leaves/{batch}/csv_tables/max_nodes_and_leaves_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

        # fig.text(0.04, 0.5, 'Traveled path (m)', va='center', rotation='vertical')
        # plt.xticks(x-0.5, config_labels, rotation=45)

        # fig.tight_layout()
        #set left, bottom, right, top, wspace, hspace
        # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        #only keep unique labels in legend
        # handles, labels = ax[0].get_legend_handles_labels()
        # unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.9, 0.1), fontsize=8)
        
        # plt.subplots_adjust(left=0.08, bottom=0.1, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # #set plot size
        # fig.set_size_inches(14, 25)
        # # plt.show()
        # plt.savefig(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png')
        # #export counts to txt file, table format
        # with open(f'battery_usage/{batch}/battery_usage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.txt', 'w') as f:
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

def plot_collisions_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/collisions/'
    if not os.path.exists(dir):
        print(f'Directory {dir} does not exist')
        return
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        average_obstacle = {}
        average_agent = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue

            if 'std' in column:
                continue

            n_agents = column.split('_')[-3]
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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            if column.endswith('obstacle'):
                average_obstacle_collisions = data[column].mean() #should be only 1 value
                if agents_string not in average_obstacle:
                    average_obstacle[agents_string] = {}
                average_obstacle[agents_string][lbl] = average_obstacle_collisions
            elif column.endswith('agent'):
                average_agent_collisions = data[column].mean()
                if agents_string not in average_agent:
                    average_agent[agents_string] = {}
                average_agent[agents_string][lbl] = average_agent_collisions


        average_collisions_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]

            collisions_stats = {}
            obstacle_collisions_for_n_agents = list(average_obstacle[n_agents_string].values())
            agent_collisions_for_n_agents = list(average_agent[n_agents_string].values())
            # max_nodes_leaves_stats['min max nodes'] = min(max_nodes_for_n_agents)
            # max_nodes_leaves_stats['median max nodes'] = np.median(max_nodes_for_n_agents)
            # max_nodes_leaves_stats['max max nodes'] = max(max_nodes_for_n_agents)
            collisions_stats['mean obstacle'] = mean(obstacle_collisions_for_n_agents)
            collisions_stats['Standard Deviation obstacle'] = np.std(obstacle_collisions_for_n_agents)
            # max_nodes_leaves_stats['min max leaves'] = min(max_leaves_for_n_agents)
            # max_nodes_leaves_stats['median max leaves'] = np.median(max_leaves_for_n_agents)
            # max_nodes_leaves_stats['max max leaves'] = max(max_leaves_for_n_agents)
            collisions_stats['mean agent'] = mean(agent_collisions_for_n_agents)
            collisions_stats['Standard Deviation agent'] = np.std(agent_collisions_for_n_agents)

            average_collisions_stats_per_n_agents[n_agents_string] = collisions_stats

        collisions_for_n_agents_df = pd.DataFrame.from_dict(average_collisions_stats_per_n_agents, orient='index').reset_index()
        collisions_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/collisions/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/collisions/{batch}/csv_tables')
        collisions_for_n_agents_df.to_csv(f'results_{algorithm_dir}/collisions/{batch}/csv_tables/collisions_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

def plot_mission_time_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/mission_time/'
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        average_mission_times = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            average_mission_time = data[column].mean() #should be only 1 value
            if agents_string not in average_mission_times:
                average_mission_times[agents_string] = {}
            average_mission_times[agents_string][lbl] = average_mission_time

        average_mission_time_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)


            max_traveled_path = 0
            max_traveled_path_column = ''
            min_traveled_path = 999
            min_traveled_path_column = ''
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
                traveled_path = data[column].to_numpy()[-1]
                if traveled_path > max_traveled_path:
                    max_traveled_path = traveled_path
                    max_traveled_path_column = config
                if traveled_path < min_traveled_path:
                    min_traveled_path = traveled_path
                    min_traveled_path_column = config

    
            mission_time_stats = {}
            mission_time_for_n_agents = list(average_mission_times[n_agents_string].values())
            # traveled_path_stats['min'] = min(traveled_path_for_n_agents)
            # traveled_path_stats['median'] = np.median(traveled_path_for_n_agents)
            # traveled_path_stats['max'] = max(traveled_path_for_n_agents)
            mission_time_stats['mean'] = mean(mission_time_for_n_agents)
            mission_time_stats['Standard Deviation'] = np.std(mission_time_for_n_agents)

            average_mission_time_stats_per_n_agents[n_agents_string] = mission_time_stats


        mission_time_for_n_agents_df = pd.DataFrame.from_dict(average_mission_time_stats_per_n_agents, orient='index').reset_index()
        mission_time_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/mission_time/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/mission_time/{batch}/csv_tables')
        mission_time_for_n_agents_df.to_csv(f'results_{algorithm_dir}/mission_time/{batch}/csv_tables/mission_time_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

def plot_return_rate_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/return_rate/'
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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue


        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[env_map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/len(agents)) #-2 because of time and first unnamed
        print("n_colors:", len(columns)-1, '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        colors = generate_high_contrast_hsv_colors(n_configs)
        i_irrelevant = 0
        
        x = np.arange(n_configs)
        config_labels = []
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        total_min = 999999999
        total_max = 0
        average_return_rates = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
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

            # color = colors[color_index]
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
            if lbl not in config_labels:
                config_labels.append(lbl)
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].bar(color_index, data[column].to_numpy()[-1], color='green')
            # for v in data[column].to_numpy():
            #     ax[agents.index(agents_string)].text(color_index, data[column][0]*0.9, f'{v:.2f}', ha='center', va='bottom', fontsize=8)
            
            total_min = min(total_min, data[column][0])
            total_max = max(total_max, data[column][0])

            average_return_rate = data[column].mean() #should be only 1 value
            if agents_string not in average_return_rates:
                average_return_rates[agents_string] = {}
            average_return_rates[agents_string][lbl] = average_return_rate

        average_return_rate_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)


            max_traveled_path = 0
            max_traveled_path_column = ''
            min_traveled_path = 999
            min_traveled_path_column = ''
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
                traveled_path = data[column].to_numpy()[-1]
                if traveled_path > max_traveled_path:
                    max_traveled_path = traveled_path
                    max_traveled_path_column = config
                if traveled_path < min_traveled_path:
                    min_traveled_path = traveled_path
                    min_traveled_path_column = config

    
            return_rate_stats = {}
            return_rate_for_n_agents = list(average_return_rates[n_agents_string].values())
            # traveled_path_stats['min'] = min(traveled_path_for_n_agents)
            # traveled_path_stats['median'] = np.median(traveled_path_for_n_agents)
            # traveled_path_stats['max'] = max(traveled_path_for_n_agents)
            return_rate_stats['mean'] = mean(return_rate_for_n_agents)
            return_rate_stats['Standard Deviation'] = np.std(return_rate_for_n_agents)

            average_return_rate_stats_per_n_agents[n_agents_string] = return_rate_stats


        return_rate_for_n_agents_df = pd.DataFrame.from_dict(average_return_rate_stats_per_n_agents, orient='index').reset_index()
        return_rate_for_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #export to csv
        if not os.path.exists(f'results_{algorithm_dir}/return_rate/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/return_rate/{batch}/csv_tables')
        return_rate_for_n_agents_df.to_csv(f'results_{algorithm_dir}/return_rate/{batch}/csv_tables/return_rate_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)


configs = []

#get all files in 'implementation_and_examples/agent_implementation/configs/{batch}'
for file in os.listdir(f'implementation_and_examples/agent_implementation/configs/{batch}'):
    config_name = file.split('.')[0]
    configs.append(config_name)    

usb_drive = f'/media/hugo/Thesis_Data/{algorithm_dir}/'
zipfiles = []
completed_per_zip = []
# for all files in the usb drive
for file in os.listdir(usb_drive):
    if file.endswith(".zip"):
        if file =='experiment_results_afternoon_11-03.zip':
            continue
        zip_file = usb_drive + file
        zipfiles.append(zip_file)

#if directory  does not exist, create it
if not os.path.exists(f'results_{algorithm_dir}/traveled_path'):
    os.makedirs(f'results_{algorithm_dir}/traveled_path')
if not os.path.exists(f'results_{algorithm_dir}/traveled_path/{batch}'):
    os.makedirs(f'results_{algorithm_dir}/traveled_path/{batch}')

if not os.path.exists(f'results_{algorithm_dir}/battery_usage'):
    os.makedirs(f'results_{algorithm_dir}/battery_usage')
if not os.path.exists(f'results_{algorithm_dir}/battery_usage/{batch}'):
    os.makedirs(f'results_{algorithm_dir}/battery_usage/{batch}')





completed_experiments, categories_and_values = get_values_for_each_category(configs)   
# check_if_all_required_experiments_done(completed_experiments, categories_and_values)
# plot_certainty_with_different_configs(usb_drive, categories_and_values)
# plot_battery_usage_with_different_configs(usb_drive, categories_and_values)
# plot_traveled_path_with_different_configs(usb_drive, categories_and_values)
# plot_bytes_sent_received_with_different_configs(usb_drive, categories_and_values)
# plot_max_nodes_leaves_with_different_configs(usb_drive, categories_and_values)
plot_collisions_with_different_configs(usb_drive, categories_and_values)
plot_mission_time_with_different_configs(usb_drive, categories_and_values)
plot_return_rate_with_different_configs(usb_drive, categories_and_values)