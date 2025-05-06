from zipfile import ZipFile, Path
import pandas as pd
import os
import re
import csv
import matplotlib.pyplot as plt
from functools import lru_cache
import numpy as np
import time

ticks_per_second = 16

batch = 'dynam_and_evap'

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
        f"_max_frontier_regions_{format_value(max_frontier_regions)}"
        f"_evaporation_time_{format_value(evaporation_time)}"
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
            end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_regions, max_route_length = parse_experiment_string(config)
            
    
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
        # if map == 'museum' or map == 'museum_tilted':
        #     continue
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


@lru_cache(maxsize=10)  # Adjust cache size as needed
def get_zip(zip_path):
    return ZipFile(zip_path, "r")


def export_average_certainty_coverage_nodes_with_different_configs(usb_drive, zipfiles, categories_and_values):
    dir_certainty = f'{usb_drive}averaged_data/'+batch+'/certainty/'
    if not os.path.exists(dir_certainty):
        os.makedirs(dir_certainty)
    dir_coverage = f'{usb_drive}averaged_data/'+batch+'/coverage/'
    if not os.path.exists(dir_coverage):
        os.makedirs(dir_coverage)
    dir_nodes = f'{usb_drive}averaged_data/'+batch+'/nodes_and_leaves/'
    if not os.path.exists(dir_nodes):
        os.makedirs(dir_nodes)
    dir_max_nodes = f'{usb_drive}averaged_data/'+batch+'/max_nodes_and_leaves/'
    if not os.path.exists(dir_max_nodes):
        os.makedirs(dir_max_nodes)
    dir_traveled_path = f'{usb_drive}averaged_data/'+batch+'/traveled_path/'
    if not os.path.exists(dir_traveled_path):
        os.makedirs(dir_traveled_path)
    dir_battery_usage = f'{usb_drive}averaged_data/'+batch+'/battery_usage/'
    if not os.path.exists(dir_battery_usage):
        os.makedirs(dir_battery_usage)
    dir_mission_time = f'{usb_drive}averaged_data/'+batch+'/mission_time/'
    if not os.path.exists(dir_mission_time):
        os.makedirs(dir_mission_time)
    dir_distance_to_deployment_site = f'{usb_drive}averaged_data/'+batch+'/return_rate/'
    if not os.path.exists(dir_distance_to_deployment_site):
        os.makedirs(dir_distance_to_deployment_site)
    dir_collisions = f'{usb_drive}averaged_data/'+batch+'/collisions/'
    if not os.path.exists(dir_collisions):
        os.makedirs(dir_collisions)
    dir_bytes_sent_received = f'{usb_drive}averaged_data/'+batch+'/bytes_sent_received/'
    if not os.path.exists(dir_bytes_sent_received):
        os.makedirs(dir_bytes_sent_received)

    for map, map_values in categories_and_values.items():

        end_times = sorted(map_values['end_time'])
        noises = sorted(map_values['noise'])
        comm_ranges = sorted(map_values['comm_range'])
        message_loss_probabilities = sorted(map_values['message_loss_probability'])
        frontier_search_radii = sorted(map_values['frontier_search_radius'])
        evaporation_times = sorted(map_values['evaporation_time'])
        max_frontier_regions = sorted(map_values['max_frontier_regions'])
        max_route_lengths = sorted(map_values['max_route_length'])
        agents = sorted(map_values['agents'], key=lambda x: int(x.split('_')[0]))
        # agent_linestyles = ['dotted', 'dashed', (5, (10, 3)), 'dashdot', 'solid']
        spawn_times = sorted(map_values['spawn_time'], key=lambda x: int(x.split('_')[-1]))
        seeds = sorted(map_values['seed'])
        for end_time in end_times:
            for spawn_time in spawn_times:
                for noise in noises:
                    print("busy with map:", map, "spawn time:", spawn_time, "noise:", noise)
                    datafile_certainty = f'{dir_certainty}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_coverage = f'{dir_coverage}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_nodes = f'{dir_nodes}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_max_nodes = f'{dir_max_nodes}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_traveled_path = f'{dir_traveled_path}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_battery_usage = f'{dir_battery_usage}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_mission_time = f'{dir_mission_time}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_distance_to_deployment_site = f'{dir_distance_to_deployment_site}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_collisions = f'{dir_collisions}{map}_{spawn_time}_noise_{noise}.csv'
                    datafile_bytes_sent_received = f'{dir_bytes_sent_received}{map}_{spawn_time}_noise_{noise}.csv'
                    
                    if os.path.exists(datafile_certainty) and os.path.exists(datafile_coverage) and os.path.exists(datafile_nodes) and os.path.exists(datafile_max_nodes) and os.path.exists(datafile_traveled_path) and os.path.exists(datafile_battery_usage) and os.path.exists(datafile_mission_time) and os.path.exists(datafile_distance_to_deployment_site) and os.path.exists(datafile_collisions) and os.path.exists(datafile_bytes_sent_received):
                        continue

                    # data_for_map_spawn_time_noise_certainty = pd.DataFrame()
                    # data_for_map_spawn_time_noise_coverage = pd.DataFrame()
                    # data_for_map_spawn_time_noise_nodes = pd.DataFrame()
                    # data_for_map_spawn_time_noise_traveled_path = pd.DataFrame()
                    # data_for_map_spawn_time_noise_battery_usage = pd.DataFrame()
                    # data_for_map_spawn_time_noise_mission_time = pd.DataFrame()
                    # data_for_map_spawn_time_noise_distance_to_deployment_site = pd.DataFrame()
                    # data_for_map_spawn_time_noise_collisions = pd.DataFrame()
                    data_for_map_spawn_time_noise_certainty = {}
                    data_for_map_spawn_time_noise_coverage = {}
                    data_for_map_spawn_time_noise_nodes = {}
                    data_for_map_spawn_time_noise_max_nodes = {}
                    data_for_map_spawn_time_noise_traveled_path = {}
                    data_for_map_spawn_time_noise_battery_usage = {}
                    data_for_map_spawn_time_noise_mission_time = {}
                    data_for_map_spawn_time_noise_distance_to_deployment_site = {}
                    data_for_map_spawn_time_noise_collisions = {}
                    data_for_map_spawn_time_noise_bytes_sent_received = {}

                    # fig,ax1 = plt.subplots(len(agents),1)
                    color = 'tab:red'

                    # ax2 = ax1.twinx()
                    # color = 'tab:blue'
                    # ax2.set_ylabel('Coverage', color=color)
                    # ax2.tick_params(axis='y', labelcolor=color)
                    # fig.tight_layout()
                    n_colors = len(frontier_search_radii) * len(max_frontier_regions) * len(max_route_lengths)
                    colors = plt.cm.viridis(np.linspace(0, 1, n_colors))
                    for message_loss_probability in message_loss_probabilities: #constant
                        for comm_range in comm_ranges: #constant
                        # for evaporation_time in evaporation_times: #is constant
                        # message_loss_probability = message_loss_probabilities[0]
                        # comm_range = comm_ranges[0]
                            # evaporation_time = evaporation_times[0]
                            for evaporation_time in evaporation_times:
                                for frontier_search_radius in frontier_search_radii: 
                                        for max_frontier_region in max_frontier_regions:
                                            for max_route_length in max_route_lengths:
                                                # color = colors[frontier_search_radii.index(frontier_search_radius) * len(max_frontier_regions) * len(max_route_lengths) + max_frontier_regions.index(max_frontier_region) * len(max_route_lengths) + max_route_lengths.index(max_route_length)]
                                                for agent in agents:
                                                    #certainty
                                                    average_certainty_all = []
                                                    average_certainty_free = []
                                                    average_certainty_occupied = []
                                                    time_certainty = []

                                                    #coverage
                                                    average_coverage = []
                                                    time_coverage = []

                                                    #nodes
                                                    average_nodes = []
                                                    average_leaves = []
                                                    time_nodes = []

                                                    average_max_nodes = []
                                                    average_max_leaves = []

                                                    average_traveled_path = []
                                                    average_battery_usage = []
                                                    average_mission_time = []
                                                    average_distance_to_deployment_site = []
                                                    average_agent_obstacle_collisions = []
                                                    average_agent_agent_collisions = []

                                                    average_bytes_sent = []
                                                    average_bytes_received = []

                                                    config = create_experiment_string(
                                                        end_time, noise, comm_range, message_loss_probability,
                                                        frontier_search_radius, evaporation_time, max_frontier_region, max_route_length
                                                    )
                                                    # average_coverage
                                                    for seed in seeds:
                                                        exp = f'{map}/{prefix}{config}/{spawn_time}/{agent}/{seed}'

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
                                                        if not found:
                                                            print("not found:", exp)
                                                            continue
                                                        zf = get_zip(experiment_is_in_zipfile)
                                                        all_files = zf.namelist()
                                                        # directories = set("/".join(f.split("/")[:-1]) for f in all_files if "/" in f)
                                                        
                                                        #CERTAINTY
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
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            df.fillna(method='ffill', inplace=True)
                                                            
                                                            # average certainty of all columns starting with 'all'
                                                            df['all'] = df[[column for column in df.columns if column.startswith('all')]].mean(axis=1)                        

                                                            # average certainty of all columns starting with 'free'
                                                            df['free'] = df[[column for column in df.columns if column.startswith('free')]].mean(axis=1)
                                                            # average certainty of all columns starting with 'occupied'
                                                            df['occupied'] = df[[column for column in df.columns if column.startswith('occupied')]].mean(axis=1)
                                                            
                                                            average_certainty_all.append(df['all'])
                                                            average_certainty_free.append(df['free'])
                                                            average_certainty_occupied.append(df['occupied'])
                                                            # print("added to average")
                                                            if len(df['tick']) > len(time_certainty):
                                                                time_certainty = df['tick'].to_numpy()/ticks_per_second
                                                        
                                                        #COVERAGE
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/coverage.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: coverage.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            df.fillna(method='ffill', inplace=True)
                                                            
                                                            # average coverage of all columns starting with 'pipuck'
                                                            df['coverage'] = df[[column for column in df.columns if column.startswith('pipuck')]].mean(axis=1)

                                                            average_coverage.append(df['coverage'])
                                                            # print("added to average")
                                                            if len(df['tick']) > len(time_coverage):
                                                                time_coverage = df['tick'].to_numpy()/ticks_per_second

                                                        #NODES
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/number_of_cells_and_leaves.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: number_of_cells_and_leaves.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            df.fillna(method='ffill', inplace=True)

                                                            # MAX NODES
                                                            max_nodes = df[[column for column in df.columns if column.startswith('cells')]].max(axis=0)
                                                            max_leaves = df[[column for column in df.columns if column.startswith('leaves')]].max(axis=0)
                                                            average_max_nodes.append(max_nodes.mean(axis=0))
                                                            average_max_leaves.append(max_leaves.mean(axis=0))

                                                            # average nodes of all columns starting with 'nodes'
                                                            df['nodes'] = df[[column for column in df.columns if column.startswith('cells')]].mean(axis=1)
                                                            df['leaves'] = df[[column for column in df.columns if column.startswith('leaves')]].mean(axis=1)

                                                            average_nodes.append(df['nodes'])
                                                            average_leaves.append(df['leaves'])
                                                            # print("added to average")
                                                            if len(df['tick']) > len(time_nodes):
                                                                time_nodes = df['tick'].to_numpy()/ticks_per_second

                                                            

                                                        #TRAVELED PATH
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/traveled_path.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: traveled_path.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            
                                                            # average traveled path of all columns starting with 'pipuck'
                                                            average_traveled_path.append(df['traveled_path'].mean(axis=0))

                                                        # BATTERY USAGE
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/battery_usage.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: battery_usage.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            
                                                            # average traveled path of all columns starting with 'pipuck'

                                                            average_battery_usage.append(df['battery_usage'].mean(axis=0))

                                                        # MISSION TIME
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/mission_time.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: mission_time.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            
                                                            # average traveled path of all columns starting with 'pipuck'
                                                            average_mission_time.append(df['mission_time'].mean(axis=0))

                                                        # DISTANCE TO DEPLOYMENT SITE
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/distance_to_deployment_site.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: distance_to_deployment_site.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)  

                                                            df['returned'] = df['distance_to_deployment_site'] < 2
                                                            n_returned = df['returned'].sum()
                                                            return_rate = n_returned / len(df['returned'])
                                                            
                                                            # average traveled path of all columns starting with 'pipuck'
                                                            average_distance_to_deployment_site.append(return_rate)
                                                        
                                                        # COLLISIONS
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/metrics.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: metrics.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            
                                                            # average traveled path of all columns starting with 'pipuck'
                                                            average_agent_obstacle_collisions.append(df['n_agent_obstacle_collisions'][0])
                                                            average_agent_agent_collisions.append(df['n_agent_agent_collisions'][0])
                                                        
                                                        # BYTES SENT AND RECEIVED
                                                        csv_path = ''
                                                        for file in all_files:
                                                            csv_temp = f"{exp}/bytes_sent_received.csv"
                                                            if file.startswith("experiment"):
                                                                csv_temp = file.split('/')[0]  + '/' + csv_temp
                                                            if file == csv_temp:
                                                                csv_path = file
                                                                break
                                                        if csv_path == '':
                                                            print("ERROR: bytes_sent_received.csv not found in", experiment_is_in_zipfile, "/", exp)
                                                            continue
                                                        with zf.open(csv_path) as csv_file:
                                                            # print("reading csv:", experiment_is_in_zipfile + '/' +csv_path)
                                                            df = pd.read_csv(csv_file)
                                                            
                                                            # average traveled path of all columns starting with 'pipuck'
                                                            average_bytes_sent.append(df['bytes_sent'].mean(axis=0))
                                                            average_bytes_received.append(df['bytes_received'].mean(axis=0))

                                                    column_name = f'evap_{evaporation_time}_{agent}'

                                                    #CERTAINTY
                                                    average_certainty_all_df = pd.DataFrame(average_certainty_all)
                                                    average_certainty_all_df.fillna(method='ffill', inplace=True)
                                                    mean_certainty_seeds_all = average_certainty_all_df.mean(axis=0).to_numpy()
                                                    
                                                    average_certainty_free_df = pd.DataFrame(average_certainty_free)
                                                    average_certainty_free_df.fillna(method='ffill', inplace=True)
                                                    mean_certainty_seeds_free = average_certainty_free_df.mean(axis=0).to_numpy()

                                                    average_certainty_occupied_df = pd.DataFrame(average_certainty_occupied)
                                                    average_certainty_occupied_df.fillna(method='ffill', inplace=True)
                                                    mean_certainty_seeds_occupied = average_certainty_occupied_df.mean(axis=0).to_numpy()

                                                    n_agents = agent.split('_')[0]
                                                    lbl = f'fsr {frontier_search_radius}, mfr {max_frontier_region}, mrl {max_route_length}'
                                                    agent_index = agents.index(agent)
                                                    # ax1[agent_index].plot(time_certainty, mean_certainty_seeds_all, label=lbl, color=color)
                                                    exp_duration_index = time_certainty.tolist().index(end_time)+1
                                                    # column_name = 
                                                    data_for_map_spawn_time_noise_certainty['time'] = time_certainty[:exp_duration_index]
                                                    data_for_map_spawn_time_noise_certainty[f'{column_name}_all'] = mean_certainty_seeds_all[:exp_duration_index]
                                                    data_for_map_spawn_time_noise_certainty[f'{column_name}_free'] = mean_certainty_seeds_free[:exp_duration_index]
                                                    data_for_map_spawn_time_noise_certainty[f'{column_name}_occupied'] = mean_certainty_seeds_occupied[:exp_duration_index]

                                                    #COVERAGE
                                                    average_coverage_df = pd.DataFrame(average_coverage)
                                                    average_coverage_df.fillna(method='ffill', inplace=True)
                                                    mean_coverage_seeds = average_coverage_df.mean(axis=0).to_numpy()
                                                    # ax1[agent_index].plot(time_coverage, mean_coverage_seeds, label=lbl, color=color)
                                                    exp_duration_index = time_coverage.tolist().index(end_time)+1
                                                    data_for_map_spawn_time_noise_coverage['time'] = time_coverage[:exp_duration_index]
                                                    data_for_map_spawn_time_noise_coverage[f'{column_name}'] = mean_coverage_seeds[:exp_duration_index]

                                                    #NODES
                                                    average_nodes_df = pd.DataFrame(average_nodes)
                                                    average_nodes_df.fillna(method='ffill', inplace=True)
                                                    mean_nodes_seeds = average_nodes_df.mean(axis=0).to_numpy()
                                                    average_leaves_df = pd.DataFrame(average_leaves)
                                                    average_leaves_df.fillna(method='ffill', inplace=True)
                                                    mean_leaves_seeds = average_leaves_df.mean(axis=0).to_numpy()
                                                    # ax1[agent_index].plot(time_nodes, mean_nodes_seeds, label=lbl, color=color)
                                                    # ax1[agent_index].plot(time_nodes, mean_leaves_seeds, label=lbl, color=color, linestyle='dashed')
                                                    exp_duration_index = time_nodes.tolist().index(end_time)+1
                                                    data_for_map_spawn_time_noise_nodes['time'] = time_nodes[:exp_duration_index]
                                                    data_for_map_spawn_time_noise_nodes[f'{column_name}_nodes'] = mean_nodes_seeds[:exp_duration_index]
                                                    data_for_map_spawn_time_noise_nodes[f'{column_name}_leaves'] = mean_leaves_seeds[:exp_duration_index]

                                                    #MAX NODES
                                                    average_max_nodes_df = pd.DataFrame(average_max_nodes)
                                                    mean_max_nodes_seeds = average_max_nodes_df.mean(axis=0).to_numpy()
                                                    average_max_leaves_df = pd.DataFrame(average_max_leaves)
                                                    mean_max_leaves_seeds = average_max_leaves_df.mean(axis=0).to_numpy()
                                                    # ax1[agent_index].plot(time_nodes, mean_max_nodes_seeds, label=lbl, color=color)
                                                    # ax1[agent_index].plot(time_nodes, mean_max_leaves_seeds, label=lbl, color=color, linestyle='dashed')
                                                    data_for_map_spawn_time_noise_max_nodes[f'{column_name}_max_nodes'] = mean_max_nodes_seeds
                                                    data_for_map_spawn_time_noise_max_nodes[f'{column_name}_max_leaves'] = mean_max_leaves_seeds

                                                    #TRAVELED PATH
                                                    average_traveled_path_df = pd.DataFrame(average_traveled_path)
                                                    mean_nodes_seeds = average_traveled_path_df.mean(axis=0).to_numpy()

                                                    data_for_map_spawn_time_noise_traveled_path[f'{column_name}'] = mean_nodes_seeds

                                                    #BATTERY USAGE
                                                    average_battery_usage_df = pd.DataFrame(average_battery_usage)
                                                    mean_nodes_seeds = average_battery_usage_df.mean(axis=0).to_numpy()

                                                    data_for_map_spawn_time_noise_battery_usage[f'{column_name}'] = mean_nodes_seeds
                                                    
                                                    #MISSION TIME
                                                    average_mission_time_df = pd.DataFrame(average_mission_time)
                                                    mean_nodes_seeds = average_mission_time_df.mean(axis=0).to_numpy()
                                                    data_for_map_spawn_time_noise_mission_time[f'{column_name}'] = mean_nodes_seeds
                                                    #DISTANCE TO DEPLOYMENT SITE
                                                    average_distance_to_deployment_site_df = pd.DataFrame(average_distance_to_deployment_site)
                                                    mean_nodes_seeds = average_distance_to_deployment_site_df.mean(axis=0).to_numpy()
                                                    data_for_map_spawn_time_noise_distance_to_deployment_site[f'{column_name}'] = mean_nodes_seeds
                                                    #COLLISIONS
                                                    average_agent_obstacle_collisions_df = pd.DataFrame(average_agent_obstacle_collisions)
                                                    mean_nodes_seeds = average_agent_obstacle_collisions_df.mean(axis=0).to_numpy()
                                                    data_for_map_spawn_time_noise_collisions[f'{column_name}_obstacle'] = mean_nodes_seeds
                                                    average_agent_agent_collisions_df = pd.DataFrame(average_agent_agent_collisions)
                                                    mean_nodes_seeds = average_agent_agent_collisions_df.mean(axis=0).to_numpy()
                                                    data_for_map_spawn_time_noise_collisions[f'{column_name}_agent'] = mean_nodes_seeds

                                                    #BYTES SENT AND RECEIVED
                                                    average_bytes_sent_df = pd.DataFrame(average_bytes_sent)
                                                    mean_nodes_seeds = average_bytes_sent_df.mean(axis=0).to_numpy()
                                                    data_for_map_spawn_time_noise_bytes_sent_received[f'{column_name}_bytes_sent'] = mean_nodes_seeds
                                                    average_bytes_received_df = pd.DataFrame(average_bytes_received)
                                                    mean_nodes_seeds = average_bytes_received_df.mean(axis=0).to_numpy()
                                                    data_for_map_spawn_time_noise_bytes_sent_received[f'{column_name}_bytes_received'] = mean_nodes_seeds

                    #create dataframes
                    data_for_map_spawn_time_noise_certainty = pd.DataFrame(data_for_map_spawn_time_noise_certainty)
                    data_for_map_spawn_time_noise_coverage = pd.DataFrame(data_for_map_spawn_time_noise_coverage)
                    data_for_map_spawn_time_noise_nodes = pd.DataFrame(data_for_map_spawn_time_noise_nodes)
                    data_for_map_spawn_time_noise_max_nodes = pd.DataFrame(data_for_map_spawn_time_noise_max_nodes)
                    data_for_map_spawn_time_noise_traveled_path = pd.DataFrame(data_for_map_spawn_time_noise_traveled_path)
                    data_for_map_spawn_time_noise_battery_usage = pd.DataFrame(data_for_map_spawn_time_noise_battery_usage)
                    data_for_map_spawn_time_noise_mission_time = pd.DataFrame(data_for_map_spawn_time_noise_mission_time)
                    data_for_map_spawn_time_noise_distance_to_deployment_site = pd.DataFrame(data_for_map_spawn_time_noise_distance_to_deployment_site)
                    data_for_map_spawn_time_noise_collisions = pd.DataFrame(data_for_map_spawn_time_noise_collisions)
                    data_for_map_spawn_time_noise_bytes_sent_received = pd.DataFrame(data_for_map_spawn_time_noise_bytes_sent_received)
                    

                    #export data to csv
                    data_for_map_spawn_time_noise_certainty.to_csv(datafile_certainty)
                    data_for_map_spawn_time_noise_coverage.to_csv(datafile_coverage)
                    data_for_map_spawn_time_noise_nodes.to_csv(datafile_nodes)
                    data_for_map_spawn_time_noise_max_nodes.to_csv(datafile_max_nodes)
                    data_for_map_spawn_time_noise_traveled_path.to_csv(datafile_traveled_path)
                    data_for_map_spawn_time_noise_battery_usage.to_csv(datafile_battery_usage)
                    data_for_map_spawn_time_noise_mission_time.to_csv(datafile_mission_time)
                    data_for_map_spawn_time_noise_distance_to_deployment_site.to_csv(datafile_distance_to_deployment_site)
                    data_for_map_spawn_time_noise_collisions.to_csv(datafile_collisions)
                    data_for_map_spawn_time_noise_bytes_sent_received.to_csv(datafile_bytes_sent_received)
    

        


overview = {}

configs = []

print("Getting all configs")
#get all files in 'implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl'
for file in os.listdir('implementation_and_examples/agent_implementation/configs/'+batch):
    config_name = file.split('.')[0]
    configs.append(config_name)    

print("Getting all files in the usb drive")
usb_drive_dirs = ['/media/hugo/Thesis_Data/CLARE/']
zipfiles = []
completed_per_zip = []
# for all files in the usb drive
for usb_drive in usb_drive_dirs:
    for file in os.listdir(usb_drive):
        if file.endswith(".zip"):
            if file =='experiment_results_afternoon_11-03.zip':
                continue
            zip_file = usb_drive + file
            zipfiles.append(zip_file)

print("Getting all unique experiments from the zip files")
completed_experiments, categories_and_values = get_values_for_each_category(configs) 
print("Checking if all required experiments are done")  
check_if_all_required_experiments_done(completed_experiments, categories_and_values)
print("Exporting average certainty, coverage, and nodes")
for usb_drive in usb_drive_dirs:
    export_average_certainty_coverage_nodes_with_different_configs(usb_drive, zipfiles, categories_and_values)
# plot_certainty_with_different_configs(usb_drive, zipfiles, categories_and_values)

# usb_drive = '/media/hugo/main/'

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

