from zipfile import ZipFile, Path
import pandas as pd
import os
import re
import csv
import matplotlib.pyplot as plt
from functools import lru_cache
import numpy as np

ticks_per_second = 16

def parse_experiment_string(s):
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
    max_frontier_cells = parsed_config['_max_frontier_cells']
    max_route_length = parsed_config['_max_route_length']
    
    return end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_cells, max_route_length

def create_experiment_string(
    end_time, noise, comm_range, message_loss_probability,
    frontier_search_radius, evaporation_time, max_frontier_cells, max_route_length
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
        f"_max_frontier_cells_{format_value(max_frontier_cells)}"
        f"_max_route_length_{format_value(max_route_length)}"
    )
    
    return config_string

def comm_loss_and_range_from_csvs(zip_file, configs):
    with ZipFile(zip_file, "r") as zf:
        results = {}
        zip_path = Path(zf)

        all_files = zf.namelist()
        directories = set("/".join(f.split("/")[:-1]) for f in all_files if "/" in f)

        # # Check if the root directory exists
        # if not os.path.exists(root_dir):
        #     raise Exception('The root directory does not exist.')

        # Iterate through directories
        for root in directories:
            if root.split('/')[-1][0] != 'S':
                    continue
            config_from_root = root.split('/')[1]

            if config_from_root not in configs:
                continue

            outer_dir = root.split('/')[1]
            exproot = root
            if outer_dir.startswith('end'):
                outer_dir = root.split('/')[0]
            else:
                experiment_results_part = root.split('/')[0] + '/'
                exproot = root.replace(experiment_results_part, '')
            

            exp = exproot.replace('implementation_and_examples/', '')

            
            # Check if 'certainty.csv' exists in the current directory
            csv_path_certainty = f"{root}/certainty.csv"
            # csv_path_coverage = f"{root}/coverage.csv"


            split_exp = exp.split('/')
            config = split_exp[1]
            spawn_time = split_exp[2]
            agents = split_exp[3]
            seed = split_exp[4]

            splitconfig = config.split('_')
            end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_cells, max_route_length = parse_experiment_string(config)
            
    
            # if csv_path_certainty in all_files:
            #     # Read the file's content
            with zf.open(csv_path_certainty) as f:
                df = pd.read_csv(f)


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
            end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_cells, max_route_length = parse_experiment_string(config)
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
            if 'max_frontier_cells' not in categories_and_values[map]:
                categories_and_values[map]['max_frontier_cells'] = set()
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
            categories_and_values[map]['max_frontier_cells'].add(max_frontier_cells)
            categories_and_values[map]['max_route_length'].add(max_route_length)
    return completed_experiments, categories_and_values

def check_if_all_required_experiments_done(completed_experiments, categories_and_values):
    #for all combinations check which configs are not in the completed experiments
    n_completed = 0
    n_non_completed = 0
    for map, map_values in categories_and_values.items():
        end_times = map_values['end_time']
        noises = map_values['noise']
        comm_ranges = map_values['comm_range']
        message_loss_probabilities = map_values['message_loss_probability']
        frontier_search_radii = map_values['frontier_search_radius']
        evaporation_times = map_values['evaporation_time']
        max_frontier_cells = map_values['max_frontier_cells']
        max_route_lengths = map_values['max_route_length']
        agents = map_values['agents']
        spawn_times = map_values['spawn_time']
        seeds = map_values['seed']
        for end_time in end_times:
            for noise in noises:
                for comm_range in comm_ranges:
                    for message_loss_probability in message_loss_probabilities:
                        for frontier_search_radius in frontier_search_radii:
                            for evaporation_time in evaporation_times:
                                for max_frontier_cell in max_frontier_cells:
                                    for max_route_length in max_route_lengths:
                                        for agent in agents:
                                            for spawn_time in spawn_times:
                                                for seed in seeds:
                                                    config = create_experiment_string(
                                                        end_time, noise, comm_range, message_loss_probability,
                                                        frontier_search_radius, evaporation_time, max_frontier_cell, max_route_length
                                                    )
                                                    exp = f'{map}/{config}/{spawn_time}/{agent}/{seed}'
                                                    if exp not in completed_experiments[map]:
                                                        print("not completed: ", exp)
                                                        n_non_completed += 1
                                                    else:
                                                        n_completed += 1
    
    print(f"Completed: {n_completed}, non-completed: {n_non_completed}")


def plot_certainty_with_different_configs(zipfiles, categories_and_values):

    for map, map_values in categories_and_values.items():
        end_times = sorted(map_values['end_time'])
        noises = sorted(map_values['noise'])
        comm_ranges = sorted(map_values['comm_range'])
        message_loss_probabilities = sorted(map_values['message_loss_probability'])
        frontier_search_radii = sorted(map_values['frontier_search_radius'])
        evaporation_times = sorted(map_values['evaporation_time'])
        max_frontier_cells = sorted(map_values['max_frontier_cells'])
        max_route_lengths = sorted(map_values['max_route_length'])
        agents = sorted(map_values['agents'], key=lambda x: int(x.split('_')[0]))
        agent_linestyles = ['dotted', 'dashed', (5, (10, 3)), 'dashdot', 'solid']
        spawn_times = sorted(map_values['spawn_time'], key=lambda x: int(x.split('_')[-1]))
        seeds = sorted(map_values['seed'])
        for end_time in end_times:
            for spawn_time in spawn_times:
                for noise in noises:
                    fig,ax1 = plt.subplots()
                    color = 'tab:red'
                    ax1.set_xlabel('Time (s)')
                    ax1.set_ylabel('Certainty', color=color)
                    ax1.tick_params(axis='y', labelcolor=color)
                    # ax2 = ax1.twinx()
                    # color = 'tab:blue'
                    # ax2.set_ylabel('Coverage', color=color)
                    # ax2.tick_params(axis='y', labelcolor=color)
                    fig.tight_layout()
                    for comm_range in comm_ranges:
                        n_colors = len(message_loss_probabilities) * len(frontier_search_radii) * len(evaporation_times) * len(max_frontier_cells) * len(max_route_lengths)
                        colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
                        for message_loss_probability in message_loss_probabilities:
                            for frontier_search_radius in frontier_search_radii:
                                color = colors[message_loss_probabilities.index(message_loss_probability) * len(frontier_search_radii) + frontier_search_radii.index(frontier_search_radius)]
                                for evaporation_time in evaporation_times:
                                    for max_frontier_cell in max_frontier_cells:
                                        for max_route_length in max_route_lengths:
                                            for agent in agents:
                                                average_all = []
                                                average_free = []
                                                average_occupied = []
                                                time_certainty = []
                                                for seed in seeds:
                                                    config = create_experiment_string(
                                                        end_time, noise, comm_range, message_loss_probability,
                                                        frontier_search_radius, evaporation_time, max_frontier_cell, max_route_length
                                                    )
                                                    exp = f'{map}/{config}/{spawn_time}/{agent}/{seed}'

                                                    for zip_file in zipfiles:
                                                        found = False
                                                        zipfilename = zip_file.split('/')[-1]
                                                        completed_in_zipfile = f"completed_in_zip/perzip/completed_experiments_unique_{map}_{zipfilename}.csv"
                                                        if not os.path.exists(completed_in_zipfile):
                                                            continue
                                                        reader = read_file(completed_in_zipfile)
                                                        for row in reader:
                                                            if exp == row[0]:
                                                                experiment_is_in_zipfile = zip_file
                                                                found = True
                                                                break
                                                        if found:
                                                            break
                                                        
                                                    with ZipFile(experiment_is_in_zipfile, "r") as zf:    
                                                        all_files = zf.namelist()
                                                        # directories = set("/".join(f.split("/")[:-1]) for f in all_files if "/" in f)

                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/certainty.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: certainty.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            df.fillna(method='ffill', inplace=True)
                                                            
                                                            # average certainty of all columns starting with 'all'
                                                            df['all'] = df[[column for column in df.columns if column.startswith('all')]].mean(axis=1)                        

                                                            # average certainty of all columns starting with 'free'
                                                            df['free'] = df[[column for column in df.columns if column.startswith('free')]].mean(axis=1)
                                                            # average certainty of all columns starting with 'occupied'
                                                            df['occupied'] = df[[column for column in df.columns if column.startswith('occupied')]].mean(axis=1)
                                                            
                                                            average_all.append(df['all'])
                                                            average_free.append(df['free'])
                                                            average_occupied.append(df['occupied'])
                                                            print("added to average")
                                                            if len(df['tick']) > len(time_certainty):
                                                                time_certainty = df['tick'].to_numpy()/ticks_per_second
                                                average_all_df = pd.DataFrame(average_all)
                                                average_all_df.fillna(method='ffill', inplace=True)
                                                s_all = average_all_df.mean(axis=0).to_numpy()
                                                n_agents = agent.split('_')[0]
                                                lbl = f'message loss probability: {message_loss_probability}, agents: {n_agents}'
                                                ax1.plot(time_certainty, s_all, label=lbl, color=color, linestyle=agent_linestyles[agents.index(agent)])
                        plt.title(f'Certainty for map {map},  noise {noise}, spawn time {spawn_time}, and communication range {comm_range}', wrap=True)
                        plt.legend(loc='upper left', fontsize=8)
                        plt.xlim(0, 400)
                        plt.ylim(0.05, 0.25)
                        plt.tight_layout()

                        plt.show()

                                                


            



        


overview = {}
#Create an overview per map, per configuration
# for map, map_exp in completed_unique.items():
#     if map not in overview:
#         overview[map] = {}
#     for exp in map_exp:
#         split_exp = exp.split('/')
#         config = split_exp[1]
#         spawn_time = split_exp[2]
#         agents = split_exp[3]
#         seed = split_exp[4]

#         if config not in overview[map]:
#             overview[map][config] = {}
    
#         if spawn_time not in overview[map][config]:
#             overview[map][config][spawn_time] = {}

#         if agents not in overview[map][config][spawn_time]:
#             overview[map][config][spawn_time][agents] = []

#         overview[map][config][spawn_time][agents].append(seed)
#         overview[map][config][spawn_time][agents] = sorted(overview[map][config][spawn_time][agents])



configs = []

#get all files in 'implementation_and_examples/agent_implementation/configs/comm_range_and_loss'
for file in os.listdir('implementation_and_examples/agent_implementation/configs/comm_range_and_loss'):
    config_name = file.split('.')[0]
    configs.append(config_name)    

usb_drive = '/media/hugo/main/'
zipfiles = []
completed_per_zip = []
# for all files in the usb drive
for file in os.listdir(usb_drive):
    if file.endswith(".zip"):
        if file =='experiment_results_afternoon_11-03.zip':
            continue
        zip_file = usb_drive + file
        zipfiles.append(zip_file)

completed_experiments, categories_and_values = get_values_for_each_category(configs)   
check_if_all_required_experiments_done(completed_experiments, categories_and_values)
plot_certainty_with_different_configs(zipfiles, categories_and_values)

usb_drive = '/media/hugo/main/'

#for all files in the usb drive
# for file in os.listdir(usb_drive):
#     if file.endswith(".zip"):
#         if file =='experiment_results_afternoon_11-03.zip':
#             continue
#         zip_file = usb_drive + file
#         print('Processing:', zip_file)
#         comm_loss_and_range_from_csvs(zip_file, configs)
#         # for outer_dir, files in certainty_files.items():
#         #     if outer_dir not in completed_experiments:
#         #         completed_experiments[outer_dir] = []
#         #     completed_experiments[outer_dir].extend(files)
#         #     completed_total += len(files)
#         # for outer_dir, files in non_completed_experiments.items():
#         #     if outer_dir not in non_completed_experiments_all:
#         #         non_completed_experiments_all[outer_dir] = []
#         #     non_completed_experiments_all[outer_dir].extend(files)
#         #     non_completed_total += len(files)
#     # if non_completed_total > 0:
#         # continue

