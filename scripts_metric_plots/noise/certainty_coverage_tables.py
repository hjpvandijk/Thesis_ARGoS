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

def plot_certainty_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/{batch}/certainty/'
    if not os.path.exists(dir):
        os.makedirs(dir)
    #for all csv files in the directory
    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        splitted = filename.split('_')
        map = splitted[0]
        if splitted[1] == 'tilted':
            map += '_tilted'
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
        time = data['time']
        
        agents = sorted(categories_and_values[map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig_all,ax_all = plt.subplots(len(agents),1)
        # fig_free,ax_free = plt.subplots(len(agents),1)
        # fig_occupied,ax_occupied = plt.subplots(len(agents),1)

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
        average_average_certainties= {}
        average_certanties_at_end_time = {}
        for i,column in enumerate(columns):
            # thisax = ax_all
            if column == 'time' or column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
            if column.endswith('free'):
                # thisax = ax_free
                data[column] -= 0.5
                max_free = max(max_free, data[column].max())
                min_free = min(min_free, data[column].min())
            elif column.endswith('occupied'):
                # thisax = ax_occupied
                data[column] = 0.5-data[column]
                max_occupied = max(max_occupied, data[column].max())
                min_occupied = min(min_occupied, data[column].min())
            elif column.endswith('all'):
                max_all = max(max_all, data[column].max())
                min_all = min(min_all, data[column].min())

            n_agents = column.split('_')[-3]
            agents_string = n_agents + '_agents'
            
            config = '_'.join(column.split('_')[0:6])

            #make sure the color is the same for all three certainty types
            #also make sure same config has same color, between the different agents
            color_index = hash(config) % n_colors
            # color = colors[color_index]
            #label is column without agent number
            lbl = column.split('_')[:-3]
            lbl = '_'.join(lbl)
            # thisax[agents.index(agents_string)].plot(time.to_numpy(), data[column].to_numpy(), label=lbl, color=color)
            average_average_certainty = data[column].mean()
            if agents_string not in average_average_certainties:
                average_average_certainties[agents_string] = {}
            average_average_certainties[agents_string][column] = average_average_certainty

            average_certainty_at_end_time = data[column].iloc[-1]
            if agents_string not in average_certanties_at_end_time:
                average_certanties_at_end_time[agents_string] = {}
            average_certanties_at_end_time[agents_string][column] = average_certainty_at_end_time

        AAC_stats_per_n_agents = {}
        end_time_certainty_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for thisax in [ax_all, ax_free, ax_occupied]:
                # for t in data['time']:
                #     thisax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
                # thisax[agents.index(n_agents_string)].set_xlabel('')
                # thisax[agents.index(n_agents_string)].set_ylabel('Certainty')
                # thisax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
                # thisax[agents.index(n_agents_string)].set_xlim(0, 400)
                # thisax[agents.index(n_agents_string)].set_ylim(0.05, 0.25)
            
            end_certainty_stats = {}
            end_certanties_for_n_agents =  average_certanties_at_end_time[n_agents_string].items()
            end_certainties_for_n_agents_free = []
            end_certainties_for_n_agents_occupied = []
            end_certainties_for_n_agents_all = []

            for key, value in end_certanties_for_n_agents:
                if key.endswith('free'):
                    end_certainties_for_n_agents_free.append(value)
                elif key.endswith('occupied'):
                    end_certainties_for_n_agents_occupied.append(value)
                elif key.endswith('all'):
                    end_certainties_for_n_agents_all.append(value)
                
            end_certainty_stats["free"] = (mean(end_certainties_for_n_agents_free))
            # end_certainty_stats["min free"] = (min(end_certainties_for_n_agents_free))
            # end_certainty_stats["median free"] = (np.median(end_certainties_for_n_agents_free))
            # end_certainty_stats["max free"] = (max(end_certainties_for_n_agents_free))
            end_certainty_stats["Standard Deviation free"] = (np.std(end_certainties_for_n_agents_free))
            end_certainty_stats["occupied"] = (mean(end_certainties_for_n_agents_occupied))
            # end_certainty_stats["min occupied"] = (min(end_certainties_for_n_agents_occupied))
            # end_certainty_stats["median occupied"] = (np.median(end_certainties_for_n_agents_occupied))
            # end_certainty_stats["max occupied"] = (max(end_certainties_for_n_agents_occupied))
            end_certainty_stats["Standard Deviation occupied"] = (np.std(end_certainties_for_n_agents_occupied))
            end_certainty_stats["all"] = (mean(end_certainties_for_n_agents_all))
            # end_certainty_stats["min all"] = (min(end_certainties_for_n_agents_all))
            # end_certainty_stats["median all"] = (np.median(end_certainties_for_n_agents_all))
            # end_certainty_stats["max all"] = (max(end_certainties_for_n_agents_all))
            end_certainty_stats["Standard Deviation all"] = (np.std(end_certainties_for_n_agents_all))

            end_time_certainty_stats_per_n_agents[n_agents_string] = end_certainty_stats
            
            AAC_stats = {}

            avg_certainties_for_n_agents =  average_average_certainties[n_agents_string].items()
            avg_certainties_for_n_agents_free = []
            avg_certainties_for_n_agents_occupied = []
            avg_certainties_for_n_agents_all = []
            for key, value in avg_certainties_for_n_agents:
                if key.endswith('free'):
                    avg_certainties_for_n_agents_free.append(value)
                elif key.endswith('occupied'):
                    avg_certainties_for_n_agents_occupied.append(value)
                elif key.endswith('all'):
                    avg_certainties_for_n_agents_all.append(value)
            AAC_stats["free"] = (mean(avg_certainties_for_n_agents_free))
            # AAC_stats["min free"] = (min(avg_certainties_for_n_agents_free))
            # AAC_stats["median free"] = (np.median(avg_certainties_for_n_agents_free))
            # AAC_stats["max free"] = (max(avg_certainties_for_n_agents_free))
            AAC_stats["Standard Deviation free"] = (np.std(avg_certainties_for_n_agents_free))
            AAC_stats["occupied"] = (mean(avg_certainties_for_n_agents_occupied))
            # AAC_stats["min occupied"] = (min(avg_certainties_for_n_agents_occupied))
            # AAC_stats["median occupied"] = (np.median(avg_certainties_for_n_agents_occupied))
            # AAC_stats["max occupied"] = (max(avg_certainties_for_n_agents_occupied))
            AAC_stats["Standard Deviation occupied"] = (np.std(avg_certainties_for_n_agents_occupied))
            AAC_stats["all"] = (mean(avg_certainties_for_n_agents_all))
            # AAC_stats["min all"] = (min(avg_certainties_for_n_agents_all))
            # AAC_stats["median all"] = (np.median(avg_certainties_for_n_agents_all))
            # AAC_stats["max all"] = (max(avg_certainties_for_n_agents_all))
            AAC_stats["Standard Deviation all"] = (np.std(avg_certainties_for_n_agents_all))
            
            AAC_stats_per_n_agents[n_agents_string] = AAC_stats

        if not os.path.exists(f'results_{algorithm_dir}/certainty_plots/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/certainty_plots/{batch}/csv_tables')


        end_certainty_stats_per_n_agents_df = pd.DataFrame.from_dict(end_time_certainty_stats_per_n_agents, orient='index').reset_index()
        end_certainty_stats_per_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #sort based on n_agents value
        end_certainty_stats_per_n_agents_df['n_agents'] = end_certainty_stats_per_n_agents_df['n_agents'].apply(lambda x: int(x.split('_')[0]))
        end_certainty_stats_per_n_agents_df = end_certainty_stats_per_n_agents_df.sort_values(by='n_agents')
        end_certainty_stats_per_n_agents_df['n_agents'] = end_certainty_stats_per_n_agents_df['n_agents'].apply(lambda x: str(x) + '_agents')
        #export to csv
        end_certainty_stats_per_n_agents_df.to_csv(f'results_{algorithm_dir}/certainty_plots/{batch}/csv_tables/AC(T_end)_{map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

        AAC_stats_per_n_agents_df = pd.DataFrame.from_dict(AAC_stats_per_n_agents, orient='index').reset_index()
        AAC_stats_per_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #sort based on n_agents value
        AAC_stats_per_n_agents_df['n_agents'] = AAC_stats_per_n_agents_df['n_agents'].apply(lambda x: int(x.split('_')[0]))
        AAC_stats_per_n_agents_df = AAC_stats_per_n_agents_df.sort_values(by='n_agents')
        AAC_stats_per_n_agents_df['n_agents'] = AAC_stats_per_n_agents_df['n_agents'].apply(lambda x: str(x) + '_agents')
    
        #export to csv
        AAC_stats_per_n_agents_df.to_csv(f'results_{algorithm_dir}/certainty_plots/{batch}/csv_tables/AAC_{map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)


        # fig_all.tight_layout()
        # fig_all.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # fig_all.suptitle(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, all certainty', wrap=True)
        # ax_all[0].set_ylim(min_all, max_all)
        # # fig_all.legend(loc='center left', fontsize=8)
        
        # fig_free.tight_layout()
        # fig_free.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # fig_free.suptitle(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, free certainty', wrap=True)
        # ax_free[0].set_ylim(min_free, max_free)
        # # fig_free.legend(loc='center left', fontsize=8)
        
        # fig_occupied.tight_layout()
        # fig_occupied.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # fig_occupied.suptitle(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, occupied certainty', wrap=True)
        # ax_occupied[0].set_ylim(min_occupied, max_occupied)
        # # fig_occupied.legend(loc='center left', fontsize=8)
        # for a,f in [(ax_all, fig_all), (ax_free, fig_free), (ax_occupied, fig_occupied)]:
        #     #only keep unique labels in legend
        #     handles, labels = a[0].get_legend_handles_labels()
        #     unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        #     f.legend(*zip(*unique), loc='upper right', fontsize=8)
        
        # plt.show()

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
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue

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

    
        # fig,ax = plt.subplots(len(agents),1)
    
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
        coverages_at_end_time = {}
        for i,column in enumerate(columns):
            if column == 'time' or column.startswith('Unnamed'):
                i_irrelevant += 1
                continue
            data[column] /= total_area
            data[column] *= 100            


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
            agents_string = n_agents + '_agents'
            # ax[agents.index(agents_string)].plot(time.to_numpy(), data[column].to_numpy(), label=lbl, color=color)

            #get average coverage over time
            average_coverage = data[column].mean()
            if agents_string not in average_coverages:
                average_coverages[agents_string] = {}
            average_coverages[agents_string][lbl] = average_coverage

            #get coverage at end time
            coverage_at_end_time = data[column].iloc[-1]
            if agents_string not in coverages_at_end_time:
                coverages_at_end_time[agents_string] = {}
            coverages_at_end_time[agents_string][lbl] = coverage_at_end_time

        ACP_stats_per_n_agents = {}
        end_time_coverage_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # ax[agents.index(n_agents_string)].set_xlim(0, end_time_for_map(env_map))
            # ax[agents.index(n_agents_string)].set_ylim(0,100)
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
            # ax[agents.index(n_agents_string)].text(end_time*0.625, 10, f'max CP({end_time}): {max_coverage_column}: {max_coverage:.2f}%', ha='left', va='center')
            # ax[agents.index(n_agents_string)].text(end_time*0.625, 5, f'min CP({end_time}): {min_coverage_column}: {min_coverage:.2f}%', ha='left', va='center')
            
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
            # ax[agents.index(n_agents_string)].text(end_time*0.250, 10, f'max ACP: {max_average_coverage_column}: {max_average_coverage:.2f}%', ha='left', va='center')
            # ax[agents.index(n_agents_string)].text(end_time*0.250, 5, f'min ACP: {min_average_coverage_column}: {min_average_coverage:.2f}%', ha='left', va='center')
            
            # Calculate statistics for each agent configuration
            if n_agents_string not in average_coverages:
                continue

            end_coverage_stats = {}

            end_coverages_for_n_agents =  list(coverages_at_end_time[n_agents_string].values())
            end_coverage_stats["CP(T_end)"] = mean(end_coverages_for_n_agents)
            # end_coverage_stats["median"] = np.median(end_coverages_for_n_agents)
            # end_coverage_stats["min"] = (min(end_coverages_for_n_agents))
            # end_coverage_stats["max"] = (max(end_coverages_for_n_agents))
            end_coverage_stats["Standard Deviation"] = (np.std(end_coverages_for_n_agents))

            end_time_coverage_stats_per_n_agents[n_agents_string] = end_coverage_stats

            ACP_stats = {}

            avg_coverages_for_n_agents =  list(average_coverages[n_agents_string].values())
            ACP_stats["ACP"] = (mean(avg_coverages_for_n_agents))
            # ACP_stats["min"] = (min(avg_coverages_for_n_agents))
            # ACP_stats["median"] = np.median(avg_coverages_for_n_agents)
            # ACP_stats["max"] = (max(avg_coverages_for_n_agents))
            ACP_stats["Standard Deviation"] = (np.std(avg_coverages_for_n_agents))

            # Create a pandas DataFrame for the statistics
            # coverage_stats_df = pd.DataFrame(stats)
            # print(f"Coverage statistics for {n_agents_string}:\n", coverage_stats_df)
            ACP_stats_per_n_agents[n_agents_string] = ACP_stats

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

        #if not exists
        if not os.path.exists(f'results_{algorithm_dir}/coverage_plots/{batch}/csv_tables'):
            os.makedirs(f'results_{algorithm_dir}/coverage_plots/{batch}/csv_tables')

        end_coverages_per_n_agents_df = pd.DataFrame.from_dict(end_time_coverage_stats_per_n_agents, orient='index').reset_index()
        end_coverages_per_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #sort based on n_agents value
        end_coverages_per_n_agents_df['n_agents'] = end_coverages_per_n_agents_df['n_agents'].apply(lambda x: int(x.split('_')[0]))
        end_coverages_per_n_agents_df = end_coverages_per_n_agents_df.sort_values(by='n_agents')
        end_coverages_per_n_agents_df['n_agents'] = end_coverages_per_n_agents_df['n_agents'].apply(lambda x: str(x) + '_agents')
        #export to csv
        end_coverages_per_n_agents_df.to_csv(f'results_{algorithm_dir}/coverage_plots/{batch}/csv_tables/CP(T_end)_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)


        ACP_stats_per_n_agents_df = pd.DataFrame.from_dict(ACP_stats_per_n_agents, orient='index').reset_index()
        ACP_stats_per_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)
        #sort based on n_agents value
        ACP_stats_per_n_agents_df['n_agents'] = ACP_stats_per_n_agents_df['n_agents'].apply(lambda x: int(x.split('_')[0]))
        ACP_stats_per_n_agents_df = ACP_stats_per_n_agents_df.sort_values(by='n_agents')
        ACP_stats_per_n_agents_df['n_agents'] = ACP_stats_per_n_agents_df['n_agents'].apply(lambda x: str(x) + '_agents')
        #export to csv
        ACP_stats_per_n_agents_df.to_csv(f'results_{algorithm_dir}/coverage_plots/{batch}/csv_tables/ACP_{env_map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)
            #set legend
            # ax[agents.index(n_agents_string)].legend(loc='center right', fontsize=8)

        # fig.text(0.04, 0.5, 'CP', va='center', rotation='vertical')
        # #Add x-axis label
        # fig.text(0.525, 0.02, 'Time (s)', ha='center')

        # # fig.tight_layout()
        # #set left, bottom, right, top, wspace, hspace
        # # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        # #only keep unique labels in legend
        # handles, labels = ax[0].get_legend_handles_labels()
        # unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.9, 0.1), fontsize=8)
        
        # plt.subplots_adjust(left=0.08, bottom=0.04, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # #set plot size
        # fig.set_size_inches(14, 25)
        # # plt.show()
        # plt.savefig(f'coverage_plots/fsr_mfr_mrl/coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'coverage_plots/fsr_mfr_mrl/coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.png')
        # #export counts to txt file, table format
        # with open(f'coverage_plots/fsr_mfr_mrl/coverage_{env_map}_noise_{noise}_spawn_time_{spawn_time}.txt', 'w') as f:
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
        
def aggregate_metrics_per_env_map():
    results_dir = 'results_' + algorithm_dir
    aggregated_data = {}

    for metric_type in ['certainty_plots', 'coverage_plots']:
        metric_dir = os.path.join(results_dir, metric_type, f'{batch}', 'csv_tables')
        if not os.path.exists(metric_dir):
            continue

        for file in os.listdir(metric_dir):
            if not file.endswith('.csv'):
                continue

            # Extract env_map from filename
            parts = file.split('_')
            metric_name = parts[0]  # e.g., "ACP" or "CP(T_end)"
            if metric_name == 'CP(T':
                metric_name = 'CP(T_end)'
            elif metric_name == 'AC(T':
                metric_name = 'AC(T_end)'
            file_without_metric = file.replace(metric_name + '_', '')
            env_map = file_without_metric.split('_')[0]
            if file_without_metric.split('_')[1] == 'tilted':
                env_map += '_tilted'

            # Read the CSV file
            filepath = os.path.join(metric_dir, file)
            data = pd.read_csv(filepath)

            # Initialize storage for this env_map and metric
            if env_map not in aggregated_data:
                aggregated_data[env_map] = {}
            if metric_name not in aggregated_data[env_map]:
                aggregated_data[env_map][metric_name] = {}

            # Aggregate metrics per agent
            for _, row in data.iterrows():
                n_agents = row['n_agents']
                if n_agents not in aggregated_data[env_map][metric_name]:
                    aggregated_data[env_map][metric_name][n_agents] = {
                        # 'min_of_min': float('inf'),
                        # 'max_of_max': float('-inf'),
                        # 'mean_of_means': [],
                        # 'max_std_dev': float('-inf'),
                        # 'median_of_medians': []
                    }



                metrics = aggregated_data[env_map][metric_name][n_agents]
                for column in row.index:
                    if 'min' in column:
                        typename = column.replace('min ', '')
                        if typename != 'min':
                            sub_name = f'min of min {typename}' 
                        else:
                            sub_name = 'min of min'
                        if sub_name not in metrics:
                            metrics[sub_name] = float('inf')
                        metrics[sub_name] = min(metrics[sub_name], row[column])
                    elif 'max' in column:
                        typename = column.replace('max ', '')
                        if typename != 'max':
                            sub_name = f'max of max {typename}' 
                        else:
                            sub_name = 'max of max'
                        if sub_name not in metrics:
                            metrics[sub_name] = float('-inf')
                        metrics[sub_name] = max(metrics[sub_name], row[column])
                    elif 'mean' in column:
                        typename = column.replace('mean ', '')
                        if typename != 'mean':
                            sub_name = f'mean of means {typename}' 
                        else:
                            sub_name = 'mean of means'
                        if sub_name not in metrics:
                            metrics[sub_name] = []
                        metrics[sub_name].append(row[column])
                    elif 'Standard Deviation' in column:
                        typename = column.replace('Standard Deviation ', '')
                        if typename != 'Standard Deviation':
                            sub_name = f'max std dev {typename}' 
                        else:
                            sub_name = 'max std dev'
                        if sub_name not in metrics:
                            metrics[sub_name] = float('-inf')
                        metrics[sub_name] = max(metrics[sub_name], row[column])
                    elif 'median' in column:
                        typename = column.replace('median ', '')
                        if typename != 'median':
                            sub_name = f'median of medians {typename}' 
                        else:
                            sub_name = 'median of medians'
                        if sub_name not in metrics:
                            metrics[sub_name] = []
                        metrics[sub_name].append(row[column])
                        
                # # metrics['min_of_min'] = min(metrics['min_of_min'], row['min'])
                # # metrics['max_of_max'] = max(metrics['max_of_max'], row['max'])
                # print(type(metrics['mean_of_means']))
                # metrics['mean_of_means'].append(row['mean'])
                # metrics['max_std_dev'] = max(metrics['max_std_dev'], row['Standard Deviation'])
                # metrics['median_of_medians'].append(row['median'])
                # Finalize aggregation (calculate mean of means and median of medians)
    # Finalize aggregation across all files
    for env_map, metrics_data in aggregated_data.items():
        for metric_name, agents_data in metrics_data.items():
            for n_agents, metrics in agents_data.items():
                for key in metrics.keys():
                        if 'mean of means' in key and isinstance(metrics[key], list):
                            metrics[key] = mean(metrics[key])
                        elif 'median of medians' in key and isinstance(metrics[key], list):
                            metrics[key] = np.median(metrics[key])

    # Print or save aggregated data for insights
    # for env_map, metrics_data in aggregated_data.items():
    #     print(f"Aggregated metrics for {env_map}:")
    #     for metric_name, agents_data in metrics_data.items():
    #         print(f"  {metric_name}:")
    #         for n_agents, metrics in agents_data.items():
    #             print(f"    {n_agents}: {metrics}")

    #export to csv
    # Export aggregated data to CSV
    # rows = []
    # for env_map, metrics_data in aggregated_data.items():
    #     for metric_name, agents_data in metrics_data.items():
    #         for n_agents, metrics in agents_data.items():
    #             row = {'env_map': env_map, 'metric_name': metric_name, 'n_agents': n_agents}
    #             row.update(metrics)
    #             rows.append(row)
    # aggregated_df = pd.DataFrame(rows)
    # aggregated_df.to_csv('results_{algorithm_dir}/aggregated_metrics.csv', index=False)
    # Separate aggregated data for certainty and coverage
    certainty_rows = []
    coverage_rows = []

    for env_map, metrics_data in aggregated_data.items():
        for metric_name, agents_data in metrics_data.items():
            for n_agents, metrics in agents_data.items():
                row = {'env_map': env_map, 'metric_name': metric_name, 'n_agents': n_agents}
                row.update(metrics)
                if metric_name in ['AC(T_end)', 'AAC']:
                    certainty_rows.append(row)
                elif metric_name in ['CP(T_end)', 'ACP']:
                    coverage_rows.append(row)

    # Create DataFrames for certainty and coverage
    certainty_df = pd.DataFrame(certainty_rows)
    coverage_df = pd.DataFrame(coverage_rows)

    # Export to separate CSV files
    certainty_df.to_csv(f'results_{algorithm_dir}/certainty_plots/{batch}/aggregated_certainty_metrics.csv', index=False)
    coverage_df.to_csv(f'results_{algorithm_dir}/coverage_plots/{batch}/aggregated_coverage_metrics.csv', index=False)

def aggregate_metrics_per_env_map_per_noise():
    results_dir = f'results_{algorithm_dir}'
    aggregated_data = {}

    for metric_type in ['certainty_plots', 'coverage_plots']:
        metric_dir = os.path.join(results_dir, metric_type, f'{batch}', 'csv_tables')
        if not os.path.exists(metric_dir):
            continue

        for file in os.listdir(metric_dir):
            if not file.endswith('.csv'):
                continue

            # Extract env_map from filename
            parts = file.split('_')
            metric_name = parts[0]  # e.g., "ACP" or "CP(T_end)"
            if metric_name == 'CP(T':
                metric_name = 'CP(T_end)'
            elif metric_name == 'AC(T':
                metric_name = 'AC(T_end)'
            file_without_metric = file.replace(metric_name + '_', '')
            env_map = file_without_metric.split('_')[0]
            if file_without_metric.split('_')[1] == 'tilted':
                env_map += '_tilted'

            noise  = file_without_metric.split('_')[-4]
            noise = float(noise)

            # Read the CSV file
            filepath = os.path.join(metric_dir, file)
            data = pd.read_csv(filepath)

            # Initialize storage for this env_map and metric
            if env_map not in aggregated_data:
                aggregated_data[env_map] = {}
            if noise not in aggregated_data[env_map]:
                aggregated_data[env_map][noise] = {}
            # if metric_name not in aggregated_data[env_map][noise]:
            #     aggregated_data[env_map][noise][metric_name] = {}

            # Aggregate metrics per agent
            for _, row in data.iterrows():
                n_agents = row['n_agents']
                if n_agents not in aggregated_data[env_map][noise]:
                    aggregated_data[env_map][noise][n_agents] = {}


                metrics = aggregated_data[env_map][noise][n_agents]
                for column in row.index:
                    if column in ['n_agents', 'Unnamed: 0']:
                        continue
                    
                    if column not in metrics:
                        metrics[column] = []
                    metrics[column].append(row[column])
                        
                # # metrics['min_of_min'] = min(metrics['min_of_min'], row['min'])
                # # metrics['max_of_max'] = max(metrics['max_of_max'], row['max'])
                # print(type(metrics['mean_of_means']))
                # metrics['mean_of_means'].append(row['mean'])
                # metrics['max_std_dev'] = max(metrics['max_std_dev'], row['Standard Deviation'])
                # metrics['median_of_medians'].append(row['median'])
                # Finalize aggregation (calculate mean of means and median of medians)
    # Finalize aggregation across all files
    for env_map, permap in aggregated_data.items():
        for noise, pernoise in permap.items():
            # for metric_name, agents_data in pernoise.items():
            for n_agents, metrics in pernoise.items():
                for key in metrics.keys():
                    if key == 'n_agents':
                        continue
                        # if 'mean of means' in key and isinstance(metrics[key], list):
                        #     metrics[key] = mean(metrics[key])
                        # elif 'median of medians' in key and isinstance(metrics[key], list):
                        #     metrics[key] = np.median(metrics[key])
                    if isinstance(metrics[key], list):
                        if 'Standard Deviation' in key:
                            #calculate pooled std deviation
                            squared_standard_devs = [x**2 for x in metrics[key]]
                            pooled_variance = np.mean(squared_standard_devs)
                            metrics[key] = np.sqrt(pooled_variance)
                        else:
                            metrics[key] = mean(metrics[key])

    # Print or save aggregated data for insights
    # for env_map, metrics_data in aggregated_data.items():
    #     print(f"Aggregated metrics for {env_map}:")
    #     for metric_name, agents_data in metrics_data.items():
    #         print(f"  {metric_name}:")
    #         for n_agents, metrics in agents_data.items():
    #             print(f"    {n_agents}: {metrics}")

    #export to csv
    # Export aggregated data to CSV
    # rows = []
    # for env_map, permap in aggregated_data.items():
    #     for noise, pernoise in permap.items():
    #         # for metric_name, agents_data in pernoise.items():
    #         for n_agents, metrics in pernoise.items():
    #             row = {
    #                 'env_map': env_map,
    #                 'noise': noise,
    #                 'n_agents': n_agents,
    #             }
    #             row.update(metrics)
    #             rows.append(row)
    # aggregated_df = pd.DataFrame(rows)
    #change any column names with 'Standard Deviation' to 'pooled standard deviation'
    # aggregated_df.rename(columns=lambda x: x.replace('Standard Deviation', '$s_p$'), inplace=True)

    # aggregated_df.to_csv(f'results_{algorithm_dir}/accuracy_plots/{batch}/aggregated_metrics.csv', index=False)
    # Separate aggregated data for certainty and coverage
    
    certainty_rows = []
    coverage_rows = []

    for env_map, permap in aggregated_data.items():
        for noise, pernoise in permap.items():
            # for metric_name, agents_data in pernoise.items():
            for n_agents, metrics in pernoise.items():
                row = {
                    'env_map': env_map,
                    'noise': noise,
                    'n_agents': n_agents,
                }
                row.update(metrics)
                if metric_name in ['AC(T_end)', 'AAC']:
                    certainty_rows.append(row)
                elif metric_name in ['CP(T_end)', 'ACP']:
                    coverage_rows.append(row)

    # Create DataFrames for certainty and coverage
    certainty_df = pd.DataFrame(certainty_rows)
    coverage_df = pd.DataFrame(coverage_rows)

    # Export to separate CSV files
    certainty_df.to_csv(f'results_{algorithm_dir}/certainty_plots/{batch}/aggregated_certainty_metrics.csv', index=False)
    coverage_df.to_csv(f'results_{algorithm_dir}/coverage_plots/{batch}/aggregated_coverage_metrics.csv', index=False)



configs = []

#get all files in 'implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl'
for file in os.listdir(f'implementation_and_examples/agent_implementation/configs/{batch}'):
    config_name = file.split('.')[0]
    configs.append(config_name)    

usb_drive_dirs = ['/media/hugo/Thesis_Data/CLARE/']
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

#if directory  does not exist, create it
if not os.path.exists('coverage_plots'):
    os.makedirs('coverage_plots')

#if directory  does not exist, create it
if not os.path.exists('certainty_plots'):
    os.makedirs('certainty_plots')



completed_experiments, categories_and_values = get_values_for_each_category(configs)   
# check_if_all_required_experiments_done(completed_experiments, categories_and_values)
for usb_drive in usb_drive_dirs:
    plot_certainty_with_different_configs(usb_drive, categories_and_values)
    plot_coverage_with_different_configs(usb_drive, categories_and_values)
# aggregate_metrics_per_env_map()
aggregate_metrics_per_env_map_per_noise()