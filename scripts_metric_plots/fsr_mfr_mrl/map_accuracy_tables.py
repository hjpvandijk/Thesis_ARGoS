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
from statistics import mean


ticks_per_second = 16
prefix = ''

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

def end_time_for_map(map):
    if 'house' in map:
        return 400
    elif 'office' in map:
        return 600
    elif 'museum' in map:
        return 1000
    
                   
def plot_precision_recall_coverage_with_different_configs(usb_drive, categories_and_values):
    dir = f'{usb_drive}averaged_data/fsr_mfr_mrl/accuracy_first_map_relayed/'
    if not os.path.exists(dir):
        os.makedirs(dir)
    #for all csv files in the directory
    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        splitted = filename.split('_')
        map = splitted[0]
        if splitted[1] == 'tilted':
            map += '_tilted'
        spawn_time = int(splitted[-4])
        noise = splitted[-2].split('.')[0]
        #if splitted[-2] is a number, prepend it to noise with a decimal
        try:
            int(splitted[-3])
            noise = splitted[-3] + '.' + noise
        except:
            pass
        noise = float(noise)

        print("noise:", noise)
        print("spawn_time:", spawn_time)
        # if noise != 0.0:
        #     continue
        # if spawn_time != 0:
        #     continue

        # #Set area size
        # if map == 'house':
        #     if spawn_time == 0:
        #         total_area=75.299072
        # elif map == 'house_tilted':
        #     if spawn_time == 0:
        #         total_area=78.604805
        # elif map == 'office':
        #     if spawn_time == 0:
        #         total_area=173.153870
        # elif map == 'office_tilted':
        #     if spawn_time == 0:
        #         total_area=176.347188
        # elif map == 'museum':
        #     if spawn_time == 0:
        #         total_area=689.311035
        # elif map == 'museum_tilted':
        #     if spawn_time == 0:
        #         total_area=701.826904

        filepath = os.path.join(dir, filename)
    
        print("opening file:",filepath)
        data = pd.read_csv(filepath)
        columns = data.columns
        
        agents = sorted(categories_and_values[map]['agents'], key=lambda x: int(x.split('_')[0]))

    
        # fig,ax = plt.subplots(len(agents),1)
    
        # color = 'tab:red'
        n_configs = int((len(columns)-1)/3/len(agents)) #-2 because of time and first unnamed
        print("n_configs:", len(columns)-1, '/3', '/', len(agents), n_configs)
        # colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
        # colors = [plt.cm.get_cmap("Set3")(i % 12) for i in range(n_colors)]  # Categorical colors
        # colors = generate_high_contrast_hsv_colors(n_colors)
        i_irrelevant = 0
        x = np.arange(n_configs)
        bar_width = 0.25
        config_labels = []
        min_recalls = {}
        max_recalls = {}
        min_precisions = {}
        max_precisions = {}
        min_coverages = {}
        max_coverages = {}
        counts = {}
        counts['mrl'] = {}
        counts['mfr'] = {}
        counts['fsr'] = {}
        recalls = {}
        precisions = {}
        coverages = {}
        invalid_area_coverages = {}
        for i,column in enumerate(columns):
            if column.startswith('Unnamed'):
                print("irrelevant column:", column)
                i_irrelevant += 1
                continue
            data[column] *= 100
            # print("data for column:", column, data[column])

            n_agents = column.split('_')[-6]
            # print("colorindex: ", (i-i_irrelevant)%n_colors)
            config = '_'.join(column.split('_')[0:6])
            precision_or_recall_or_coverage = column.split('_')[-1]

            fsr_symbol = '$R_f$'
            fsr = config.split('_')[1] #R_f
            if int(fsr) == 99999:
                fsr     = '$\infty$'
            mfr_symbol = '$N_f$'
            mfr = config.split('_')[3] #N_f
            if int(mfr) == 99999:
                mfr = '$\infty$'
            mrl_symbol = '$N_s$'
            mrl = config.split('_')[5] #N_s
            if int(mrl) == 99999:
                mrl = '$\infty$'
            lbl = f'{fsr_symbol}={fsr}, {mfr_symbol}={mfr}, {mrl_symbol}={mrl}'

            if lbl not in config_labels:
                config_labels.append(lbl)

            #make sure the color is the same for all three certainty types
            #also make sure same config has same color, between the different agents
            if 'config_to_color_index' not in locals():
                config_to_color_index = {}
                current_color_index = 0

            if config not in config_to_color_index:
                config_to_color_index[config] = current_color_index
                current_color_index += 1

            color_index = config_to_color_index[config]
            # print("config:", config, "color_index:", color_index)
            # print("color_index:", color_index)
            # color = colors[color_index]
            # color = colors[(i-i_irrelevant)%n_colors]   
            #label is column without agent number
            # lbl = column.split('_')[:-6]
            # lbl = '_'.join(lbl)
            agents_string = n_agents + '_agents'

            if agents_string not in recalls:
                recalls[agents_string] = []
            if agents_string not in precisions:
                precisions[agents_string] = []
            if agents_string not in coverages:
                coverages[agents_string] = []
            if agents_string not in invalid_area_coverages:
                invalid_area_coverages[agents_string] = []

            # ax[agents.index(agents_string)].plot(time, data[column], label=lbl, color=color)
            # make bar plot
            if agents_string not in min_recalls:
                min_recalls[agents_string] = {}
                min_recalls[agents_string]['recall'] = 100
                min_recalls[agents_string]['config'] = config
                min_recalls[agents_string]['lbl'] = lbl
                max_recalls[agents_string] = {}
                max_recalls[agents_string]['recall'] = 0
                max_recalls[agents_string]['config'] = config
                max_recalls[agents_string]['lbl'] = lbl
                min_precisions[agents_string] = {}
                min_precisions[agents_string]['precision'] = 100
                min_precisions[agents_string]['config'] = config
                min_precisions[agents_string]['lbl'] = lbl
                max_precisions[agents_string] = {}
                max_precisions[agents_string]['precision'] = 0
                max_precisions[agents_string]['config'] = config
                max_precisions[agents_string]['lbl'] = lbl
                min_coverages[agents_string] = {}
                min_coverages[agents_string]['coverage'] = 100
                min_coverages[agents_string]['config'] = config
                min_coverages[agents_string]['lbl'] = lbl
                max_coverages[agents_string] = {}
                max_coverages[agents_string]['coverage'] = 0
                max_coverages[agents_string]['config'] = config
                max_coverages[agents_string]['lbl'] = lbl

            

            if precision_or_recall_or_coverage == 'precision':
                color = 'tab:blue'
                #plot it on one half of the bar space
                # ax[agents.index(agents_string)].bar(color_index, data[column], label='precision', color=color, width=bar_width)
                #write the value in the bar
                # for j, v in enumerate(data[column]):
                #     ax[agents.index(agents_string)].text(color_index, 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=6)
                precision = data[column][0]
                if precision < min_precisions[agents_string]['precision']:
                    min_precisions[agents_string]['precision'] = precision
                    min_precisions[agents_string]['config'] = config
                    min_precisions[agents_string]['lbl'] = lbl
                if precision > max_precisions[agents_string]['precision']:
                    max_precisions[agents_string]['precision'] = precision
                    max_precisions[agents_string]['config'] = config
                    min_precisions[agents_string]['lbl'] = lbl
                precisions[agents_string].append(precision)
            elif precision_or_recall_or_coverage == 'recall':   
                color = 'tab:orange'
                #plot it on the other half of the bar space
                # ax[agents.index(agents_string)].bar(color_index + bar_width, data[column], label='recall', color=color, width=bar_width)
                #write the value in the bar
                # for j, v in enumerate(data[column]):
                #     ax[agents.index(agents_string)].text(color_index + bar_width, 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=6)
                recall = data[column][0]
                if recall < min_recalls[agents_string]['recall']:
                    min_recalls[agents_string]['recall'] = recall
                    min_recalls[agents_string]['config'] = config
                    min_recalls[agents_string]['lbl'] = lbl
                if recall > max_recalls[agents_string]['recall']:
                    max_recalls[agents_string]['recall'] = recall
                    max_recalls[agents_string]['config'] = config
                    max_recalls[agents_string]['lbl'] = lbl
                recalls[agents_string].append(recall)
            elif precision_or_recall_or_coverage == 'coverage':
                color = 'tab:green'
                #plot it on the other half of the bar space
                # ax[agents.index(agents_string)].bar(color_index - bar_width, data[column], label='coverage', color=color, width=bar_width)
                #write the value in the bar
                # for j, v in enumerate(data[column]):
                #     ax[agents.index(agents_string)].text(color_index - bar_width, 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=6)
                coverage = data[column][0]
                if coverage < min_coverages[agents_string]['coverage']:
                    min_coverages[agents_string]['coverage'] = coverage
                    min_coverages[agents_string]['config'] = config
                    min_coverages[agents_string]['lbl'] = lbl
                if coverage > max_coverages[agents_string]['coverage']:
                    max_coverages[agents_string]['coverage'] = coverage
                    max_coverages[agents_string]['config'] = config
                    max_coverages[agents_string]['lbl'] = lbl
                coverages[agents_string].append(coverage)
            # if precision_or_recall_or_coverage == 'covered_invalid_area':
            #     invalid_area_coverage = data[column][0]
            #     invalid_area_coverages[agents_string].append(invalid_area_coverage)

        accuracy_stats_per_n_agents = {}
        for n_agents_string in agents:
            n_agents = n_agents_string.split('_')[0]
            # for t in data['time']:
            #     ax[agents.index(n_agents_string)].axvline(x=t, color='gray', linestyle='--', linewidth=0.5)
            # ax[agents.index(n_agents_string)].set_xlabel('')
            # ax[agents.index(n_agents_string)].set_ylabel('')
            # ax[agents.index(n_agents_string)].set_title(f'{n_agents} agents', wrap=True)
            # #remove xticks
            # ax[agents.index(n_agents_string)].set_xticks([])
            # # ax[agents.index(n_agents_string)].set_xlim(0,100)
            # ax[agents.index(n_agents_string)].set_ylim(0,100)
            #data at end_time
            data_end_time = data.iloc[-1,2:]

            min_recall = min_recalls[n_agents_string]['recall']
            min_recall_config = min_recalls[n_agents_string]['config']
            min_recall_lbl = min_recalls[n_agents_string]['lbl']
            max_recall = max_recalls[n_agents_string]['recall']
            max_recall_config = max_recalls[n_agents_string]['config']
            max_recall_lbl = max_recalls[n_agents_string]['lbl']
            min_precision = min_precisions[n_agents_string]['precision']
            min_precision_config = min_precisions[n_agents_string]['config']
            min_precision_lbl = min_precisions[n_agents_string]['lbl']
            max_precision = max_precisions[n_agents_string]['precision']
            max_precision_config = max_precisions[n_agents_string]['config']
            max_precision_lbl = max_precisions[n_agents_string]['lbl']
            min_coverage = min_coverages[n_agents_string]['coverage']
            min_coverage_config = min_coverages[n_agents_string]['config']
            min_coverage_lbl = min_coverages[n_agents_string]['lbl']
            max_coverage = max_coverages[n_agents_string]['coverage']
            max_coverage_config = max_coverages[n_agents_string]['config']
            max_coverage_lbl = max_coverages[n_agents_string]['lbl']

            min_recall_color_index = config_to_color_index[min_recall_config]
            max_recall_color_index = config_to_color_index[max_recall_config]
            min_precision_color_index = config_to_color_index[min_precision_config]
            max_precision_color_index = config_to_color_index[max_precision_config]
            min_coverage_color_index = config_to_color_index[min_coverage_config]
            max_coverage_color_index = config_to_color_index[max_coverage_config]
        
            #Circle the bars with min and max recall and precision
            # ax[agents.index(n_agents_string)].bar(min_precision_color_index, min_precisions[n_agents_string]['precision'], label='min precision', facecolor='none', edgecolor='deepskyblue', linewidth=4, width=bar_width)
            # ax[agents.index(n_agents_string)].bar(min_recall_color_index + bar_width, min_recalls[n_agents_string]['recall'], label='min recall', facecolor='none', edgecolor='gold', linewidth=4, width=bar_width)
            # ax[agents.index(n_agents_string)].bar(max_precision_color_index, max_precisions[n_agents_string]['precision'], label='max precision', facecolor='none', edgecolor='darkblue', linewidth=4, width=bar_width)
            # ax[agents.index(n_agents_string)].bar(max_recall_color_index + bar_width, max_recalls[n_agents_string]['recall'], label='max recall', facecolor='none', edgecolor='orangered', linewidth=4, width=bar_width)
            # ax[agents.index(n_agents_string)].bar(min_coverage_color_index - bar_width, min_coverages[n_agents_string]['coverage'], label='min coverage', facecolor='none', edgecolor='limegreen', linewidth=4, width=bar_width)
            # ax[agents.index(n_agents_string)].bar(max_coverage_color_index - bar_width, max_coverages[n_agents_string]['coverage'], label='max coverage', facecolor='none', edgecolor='darkgreen', linewidth=4, width=bar_width)

            if 'max' not in counts['fsr']:
                counts['fsr']['max'] = {}
            if 'min' not in counts['fsr']:
                counts['fsr']['min'] = {}
    
            if 'max' not in counts['mfr']:
                counts['mfr']['max'] = {}
            if 'min' not in counts['mfr']:
                counts['mfr']['min'] = {}
           
            if 'max' not in counts['mrl']:
                counts['mrl']['max'] = {}
            if 'min' not in counts['mrl']:
                counts['mrl']['min'] = {}

            #set legend
            # ax[agents.index(n_agents_string)].legend(loc='center right', fontsize=8)

            accuracy_stats = {}
            recalls_for_n_agents = recalls[n_agents_string]
            precisions_for_n_agents = precisions[n_agents_string]
            coverages_for_n_agents = coverages[n_agents_string]
            invalid_area_coverages_for_n_agents = invalid_area_coverages[n_agents_string]

            accuracy_stats["mean recall"] = mean(recalls_for_n_agents)
            accuracy_stats["min recall"] = min(recalls_for_n_agents)
            accuracy_stats["median recall"] = np.median(recalls_for_n_agents)
            accuracy_stats["max recall"] = max(recalls_for_n_agents)
            accuracy_stats["Standard Deviation recall"] = np.std(recalls_for_n_agents)
            accuracy_stats["mean precision"] = mean(precisions_for_n_agents)
            accuracy_stats["min precision"] = min(precisions_for_n_agents)
            accuracy_stats["median precision"] = np.median(precisions_for_n_agents)
            accuracy_stats["max precision"] = max(precisions_for_n_agents)
            accuracy_stats["Standard Deviation precision"] = np.std(precisions_for_n_agents)
            accuracy_stats["mean coverage"] = mean(coverages_for_n_agents)
            accuracy_stats["min coverage"] = min(coverages_for_n_agents)
            accuracy_stats["median coverage"] = np.median(coverages_for_n_agents)
            accuracy_stats["max coverage"] = max(coverages_for_n_agents)
            accuracy_stats["Standard Deviation coverage"] = np.std(coverages_for_n_agents)
            # accuracy_stats["mean invalid area coverage"] = mean(invalid_area_coverages_for_n_agents)
            # accuracy_stats["min invalid area coverage"] = min(invalid_area_coverages_for_n_agents)
            # accuracy_stats["max invalid area coverage"] = max(invalid_area_coverages_for_n_agents)
            # accuracy_stats["Standard Deviation invalid area coverage"] = np.std(invalid_area_coverages_for_n_agents)
            accuracy_stats_per_n_agents[n_agents_string] = accuracy_stats

        accuracy_stats_per_n_agents_df = pd.DataFrame.from_dict(accuracy_stats_per_n_agents, orient='index').reset_index()
        accuracy_stats_per_n_agents_df.rename(columns={'index': 'n_agents'}, inplace=True)

        #create dir if not exists
        if not os.path.exists(f'accuracy_plots/fsr_mfr_mrl/csv_tables'):
            os.makedirs(f'accuracy_plots/fsr_mfr_mrl/csv_tables')   

        accuracy_stats_per_n_agents_df.to_csv(f'accuracy_plots/fsr_mfr_mrl/csv_tables/accuracy_{map}_noise_{noise}_spawn_time_{spawn_time}.csv', index=False)

        

        # fig.text(0.04, 0.5, '%', va='center', rotation='vertical')
        #Set x ticks
        # plt.xticks(x-bar_width, config_labels, rotation=45)

        # fig.tight_layout()
        #set left, bottom, right, top, wspace, hspace
        # fig.suptitle(f'Coverage for map {map},  noise {noise}, spawn time {spawn_time}, coverage', wrap=True)
        #only keep unique labels in legend
        # handles, labels = ax[0].get_legend_handles_labels()
        # unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # fig.legend(*zip(*unique), loc='center right', bbox_to_anchor=(0.75, 0.396), fontsize=8, ncol=len(unique))
        # plt.subplots_adjust(left=0.08, bottom=0.1, right=0.96, top=0.95, hspace=0.15, wspace=0.2)
        # #set plot size
        # fig.set_size_inches(14, 25)
        # # plt.show()
        # plt.savefig(f'accuracy_plots/fsr_mfr_mrl/accuracy_fsr_mfr_mrl_{map}_noise_{noise}_spawn_time_{spawn_time}.png', dpi=300, transparent=False, bbox_inches='tight')
        # print(f'accuracy_plots/fsr_mfr_mrl/accuracy_fsr_mfr_mrl_{map}_noise_{noise}_spawn_time_{spawn_time}.png')
        
        

configs = []

#get all files in 'implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl'
for file in os.listdir('implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl'):
    config_name = file.split('.')[0]
    configs.append(config_name)    

usb_drive = '/media/hugo/Thesis_Data/CLARE/'
zipfiles = []
completed_per_zip = []
# for all files in the usb drive
for file in os.listdir(usb_drive):
    if file.endswith(".zip"):
        # if file =='experiment_results_afternoon_11-03.zip':
        #     continue
        zip_file = usb_drive + file
        zipfiles.append(zip_file)

#if directory 'accuracy' does not exist, create it
if not os.path.exists('accuracy_plots'):
    os.makedirs('accuracy_plots')

completed_experiments, categories_and_values = get_values_for_each_category(configs)   
# check_if_all_required_experiments_done(completed_experiments, categories_and_values)
# plot_certainty_with_different_configs(usb_drive, categories_and_values)
plot_precision_recall_coverage_with_different_configs(usb_drive, categories_and_values)
