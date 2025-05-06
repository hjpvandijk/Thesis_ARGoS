from zipfile import ZipFile, Path
import pandas as pd
import os
import re
import csv
import matplotlib.pyplot as plt
from functools import lru_cache
import numpy as np
import time
import io

from map_accuracy_quadtree import set_variables_based_on_map, read_arena_boxes, read_arena_spawn_boxes, create_arena_rectangles_and_circles, read_arena_cylinders, union_map, union_u_map_and_spawn_boxes, divide_map_into_grid_tree, calculate_precision_recall_coverage, get_spawn_times
import multiprocessing as mp
import concurrent.futures

ticks_per_second = 16

batch = 'comm_range_and_loss'

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
    
def read_quadtree_data(zf, filename):
    with zf.open(filename, 'r') as file:
        text_file = io.TextIOWrapper(file, encoding='utf-8')  # Decode bytes to text
        reader = csv.DictReader(text_file)
        data = [row for row in reader]
    return data



    
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

def find_log_file(zf, directory):
    all_files = zf.namelist()
    #find the log file in the directory
    for file in all_files:
        if file.startswith(directory) and file.endswith(".log"):
            return file
    return None


def process_experiment(seed, map, prefix, config, spawn_time, agent, quadtree_type, zipfiles, u_map, tree, grid):
    exp = f'{map}/{prefix}{config}/{spawn_time}/{agent}/{seed}'
    
    for zip_file in zipfiles:
        found = False
        zipfilename = zip_file.split('/')[-1]
        completed_in_zipfile = f"completed_in_zip/perzip/completed_experiments_unique_{map}_{zipfilename}.csv"
        if not os.path.exists(completed_in_zipfile):
            continue
        
        # Read the completed experiment CSV
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
        return None

    zf = get_zip(experiment_is_in_zipfile)
    all_files = zf.namelist()
    agent_int = int(agent.split('_')[0])

    
    # Find first finished agent
    csv_path = ''
    first_finished_id = ''
    for file in all_files:
        csv_temp = f"{exp}/mission_time.csv"
        if file.startswith("experiment"):
            csv_temp = file.split('/')[0] + '/' + csv_temp
        if file == csv_temp:
            csv_path = file
            break
    if csv_path == '':
        print(f"ERROR: {csv_path} not found in", experiment_is_in_zipfile)
        return None
    
    
    with zf.open(csv_path) as f:
        df = pd.read_csv(f)
        first_finished_id = df.loc[df['mission_time'].idxmin(), 'agent_id']
        first_finished_time = df.loc[df['mission_time'].idxmin(), 'mission_time']
        print("first finished id:", first_finished_id, "with time:", df.loc[df['mission_time'].idxmin(), 'mission_time'])
        
    dir = csv_path.replace('mission_time.csv', '')
    log_file = find_log_file(zf, dir)
    if log_file == '':
        print(f"ERROR: {log_file} not found in", experiment_is_in_zipfile)
        return None
    log_data = zf.open(log_file).read().decode('utf-8')
    spawn_data = get_spawn_times(log_data, ticks_per_second)

    spawn_boxes = read_arena_spawn_boxes(spawn_data, first_finished_time)
    arena_spawn_boxes, notrelevant = create_arena_rectangles_and_circles(spawn_boxes, [])
    #redo the tree with spawn boxes
    u_map = union_u_map_and_spawn_boxes(u_map, arena_spawn_boxes)

    str_tree_with_spawn_boxes, grid_with_spawn_boxes = divide_map_into_grid_tree(u_map)

    pipuck_string = first_finished_id
    csv_path = ''
    for file in all_files:
        csv_temp = f"{exp}/{quadtree_type}{pipuck_string}.csv"
        if file.startswith("experiment"):
            csv_temp = file.split('/')[0]  + '/' + csv_temp
        if file == csv_temp:
            csv_path = file
            break
    if csv_path == '':
        print(f"ERROR: {csv_path} not found in", experiment_is_in_zipfile)
        return None

    quadtree_data = read_quadtree_data(zf, csv_path)
    # print("checking for", csv_path, "in", experiment_is_in_zipfile)

    precision, recall,coverage, covered_invalid_area = calculate_precision_recall_coverage(u_map, tree, str_tree_with_spawn_boxes, grid, grid_with_spawn_boxes, quadtree_data, spawn_time, False)
    
    return precision, recall, coverage, covered_invalid_area


def export_average_precision_recall_with_different_configs(usb_drive, zipfiles, categories_and_values):
    dir_accuracy = f'{usb_drive}averaged_data/'+batch+'/accuracy_first_map_relayed/'
    if not os.path.exists(dir_accuracy):
        os.makedirs(dir_accuracy)

    for map, map_values in categories_and_values.items():
        # if map != 'museum_tilted':
        #     continue
        print("MAP:", map)
        set_variables_based_on_map(map)

        end_times = sorted(map_values['end_time'])
        noises = sorted(map_values['noise'])
        comm_ranges = sorted(map_values['comm_range'])
        message_loss_probabilities = sorted(map_values['message_loss_probability'])
        frontier_search_radii = sorted(map_values['frontier_search_radius'])
        evaporation_times = sorted(map_values['evaporation_time'])
        max_frontier_regions = sorted(map_values['max_frontier_regions'])
        max_route_lengths = sorted(map_values['max_route_length'])
        agents = sorted(map_values['agents'], key=lambda x: int(x.split('_')[0]))
        print(agents)
        # agent_linestyles = ['dotted', 'dashed', (5, (10, 3)), 'dashdot', 'solid']
        spawn_times = sorted(map_values['spawn_time'], key=lambda x: int(x.split('_')[-1]))
        seeds = sorted(map_values['seed'])

        for end_time in end_times:
            for spawn_time in spawn_times:
                # if spawn_time == 'spawn_time_0': #only spawn_time_0 for now
                #     continue
                arena_boxes = read_arena_boxes()
                arena_cylinders = read_arena_cylinders()
                arena_rectangles, arena_circles = create_arena_rectangles_and_circles(arena_boxes, arena_cylinders)
                u_map = union_map(arena_rectangles, arena_circles)
                tree, grid = divide_map_into_grid_tree(u_map)

                # if spawn_time != 'spawn_time_180':
                #     continue

                for noise in noises:
                    # if noise != 1:
                    #     continue
                    n_experiments = len(comm_ranges) * len(message_loss_probabilities) * len(frontier_search_radii) * len(evaporation_times) * len(max_frontier_regions) * len(max_route_lengths) * len(agents) * len(seeds)
                    run_experiments = 0
                    datafile = f'{dir_accuracy}{map}_{spawn_time}_noise_{noise}_.csv'
                    if os.path.exists(datafile):
                        continue
                    quadtree_for_map_spawn_time_noise = pd.DataFrame()
                    new_columns = {}

                    for message_loss_probability in message_loss_probabilities:
                        for comm_range in comm_ranges: 
                    # for evaporation_time in evaporation_times: #is constant
                    # message_loss_probability = message_loss_probabilities[0]
                    # comm_range = comm_ranges[0]
                            evaporation_time = evaporation_times[0]
                    # for frontier_search_radius in frontier_search_radii: #constant
                    #         for max_frontier_region in max_frontier_regions: #constant
                    #             for max_route_length in max_route_lengths: #constant
                            frontier_search_radius = frontier_search_radii[0]
                            max_frontier_region = max_frontier_regions[0]
                            max_route_length = max_route_lengths[0]
                            for agent in agents:
                                # if agent != '15_agents':
                                #     continue
                                for quadtree_type in ['quadtree_returning_','quadtree_finished_exploring_', 'quadtree_map_relayed_']:
                                    if quadtree_type != 'quadtree_map_relayed_':
                                        continue


                                    #certainty
                                    average_precision = []
                                    average_recall = []
                                    average_coverage = []
                                    average_covered_invalid_area = []

                                    config = create_experiment_string(
                                        end_time, noise, comm_range, message_loss_probability,
                                        frontier_search_radius, evaporation_time, max_frontier_region, max_route_length
                                    )
                                    
                                    with concurrent.futures.ProcessPoolExecutor() as executor:
                                        future_to_seed = {executor.submit(process_experiment, seed, map, prefix, config, spawn_time, agent, quadtree_type, zipfiles, u_map, tree, grid): seed for seed in seeds}
                                        for future in concurrent.futures.as_completed(future_to_seed):
                                            result = future.result()
                                            if result:
                                                precision, recall, coverage, covered_invalid_area = result
                                                average_precision.append(precision)
                                                average_recall.append(recall)
                                                average_coverage.append(coverage)
                                                average_covered_invalid_area.append(covered_invalid_area)
                                                run_experiments += 1
                                                print(f"run experiments: {run_experiments}/{n_experiments}")

                                    
                                            

                                    #ACCURACY
                                    average_precision_df = pd.DataFrame(average_precision)
                                    mean_precision_seeds_all = average_precision_df.mean(axis=0).to_numpy()
                                    
                                    average_recall_df = pd.DataFrame(average_recall)
                                    mean_recall_seeds_all = average_recall_df.mean(axis=0).to_numpy()

                                    average_coverage_df = pd.DataFrame(average_coverage)
                                    mean_coverage_seeds_all = average_coverage_df.mean(axis=0).to_numpy()

                                    average_covered_invalid_area_df = pd.DataFrame(average_covered_invalid_area)
                                    mean_covered_invalid_area_seeds_all = average_covered_invalid_area_df.mean(axis=0).to_numpy()

                                    n_agents = agent.split('_')[0]
                                    column = f'cr_{comm_range}_cl_{message_loss_probability}'
                                    agent_index = agents.index(agent)
                                    print("updating columns:" ,f'{column}_{agent}_{quadtree_type}')
                                    # quadtree_for_map_spawn_time_noise[f'{column}_{agent}_{quadtree_type}precision'] =  mean_precision_seeds_all
                                    # quadtree_for_map_spawn_time_noise[f'{column}_{agent}_{quadtree_type}recall'] = mean_recall_seeds_all
                                    new_columns[f'{column}_{agent}_{quadtree_type}precision'] = mean_precision_seeds_all
                                    new_columns[f'{column}_{agent}_{quadtree_type}recall'] = mean_recall_seeds_all
                                    new_columns[f'{column}_{agent}_{quadtree_type}coverage'] = mean_coverage_seeds_all
                                    new_columns[f'{column}_{agent}_{quadtree_type}covered_invalid_area'] = mean_covered_invalid_area_seeds_all
                    
                    quadtree_for_map_spawn_time_noise = pd.DataFrame(new_columns)
                    print("exporting to", datafile)
                    #export data to csv
                    quadtree_for_map_spawn_time_noise.to_csv(datafile) 
                    # return   

configs = []

print("Getting all configs")
#get all files in 'implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl'
for file in os.listdir('implementation_and_examples/agent_implementation/configs/'+batch):
    config_name = file.split('.')[0]
    configs.append(config_name)    

print("Getting all files in the usb drive")
usb_drive = '/media/hugo/Thesis_Data/CLARE/'
zipfiles = []
completed_per_zip = []
# for all files in the usb drive
for file in os.listdir(usb_drive):
    if file.endswith(".zip"):
        if file =='experiment_results_afternoon_11-03.zip':
            continue
        zip_file = usb_drive + file
        zipfiles.append(zip_file)

print("Getting all unique experiments from the zip files")
completed_experiments, categories_and_values = get_values_for_each_category(configs) 
# print("Checking if all required experiments are done")  
# check_if_all_required_experiments_done(completed_experiments, categories_and_values)
print("Exporting average precision and recall")
export_average_precision_recall_with_different_configs(usb_drive, zipfiles, categories_and_values)
