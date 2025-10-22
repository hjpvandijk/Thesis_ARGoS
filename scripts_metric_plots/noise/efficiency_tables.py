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
# batch = "noise"
batch = "noise_wallfollowing"

# algorithm_dir = 'CLARE'
algorithm_dir = 'CLARE_wallfollowing'

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
    
            
def calculate_exploration_efficiency():
    traveled_path_dir = f'results_{algorithm_dir}/traveled_path/{batch}/csv_tables'
    accuracy_plots_dir = f'results_{algorithm_dir}/accuracy_plots/{batch}/csv_tables'
    output_dir = f'results_{algorithm_dir}/traveled_efficiency/{batch}/csv_tables'

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Dictionary to store coverage data from accuracy plots
    coverage_data = {}
    coverage_stds = {}

    # Read accuracy plots to extract coverage data
    for file in os.listdir(accuracy_plots_dir):
        if file.endswith('.csv'):
            parts = file.split('_')
            map_name = parts[1]
            if parts[2] == 'tilted':
                map_name += '_tilted'
            noise = float(parts[-4])
            spawn_time = int(parts[-1].replace('.csv', ''))

            filepath = os.path.join(accuracy_plots_dir, file)
            data = pd.read_csv(filepath)

            for _, row in data.iterrows():
                n_agents = row['n_agents']
                coverage = row['coverage']
                coverage_std = row['Sample Standard Deviation coverage']
                key = (map_name, noise, spawn_time, n_agents)
                coverage_data[key] = coverage
                coverage_stds[key] = coverage_std

    # Read traveled path data and calculate exploration efficiency
    for file in os.listdir(traveled_path_dir):
        if file.endswith('.csv'):
            parts = file.split('_')
            map_name = parts[2]
            if parts[3] == 'tilted':
                map_name += '_tilted'
            noise = float(parts[-4])
            spawn_time = int(parts[-1].replace('.csv', ''))

            filepath = os.path.join(traveled_path_dir, file)
            data = pd.read_csv(filepath)

            efficiency_data = []

            for _, row in data.iterrows():
                n_agents = row['n_agents']
                agent_int = int(n_agents.split('_')[0])
                traveled_path = row['mean'] * agent_int
                traveled_path_std = row['Sample Standard Deviation'] * agent_int

                key = (map_name, noise, spawn_time, n_agents)
                coverage = coverage_data.get(key, None)
                coverage_std = coverage_stds.get(key, None)

                if coverage is not None:
                    exploration_efficiency = 100 * coverage / traveled_path if traveled_path > 0 else 0
                    # Calculate the standard deviation of the exploration efficiency
                    # if traveled_path > 0:
                    #     # Use the formula for propagation of uncertainty
                    #     # std_efficiency = (coverage / traveled_path) * sqrt((std_coverage / coverage)^2 + (std_traveled_path / traveled_path)^2)
                    #     std_efficiency = (coverage / traveled_path) * np.sqrt((coverage_std / coverage)**2 + (traveled_path_std / traveled_path)**2)
                    # else:
                    #     std_efficiency = 0
                    efficiency_data.append({
                        # 'map': map_name,
                        # 'noise': noise,
                        # 'spawn_time': spawn_time,
                        'n_agents': n_agents,
                        'coverage': coverage,
                        'traveled_path_sum': traveled_path,
                        'exploration_efficiency': exploration_efficiency
                    })

            # Save exploration efficiency data to CSV
            output_file = os.path.join(output_dir, f'efficiency_{map_name}_noise_{noise}_spawn_time_{spawn_time}.csv')
            pd.DataFrame(efficiency_data).to_csv(output_file, index=False)
            print(f"Saved exploration efficiency data for {map_name}, noise: {noise}, spawn_time: {spawn_time} to {output_file}")

    print(f"Exploration efficiency data saved to {output_dir}")

def calculate_battery_exploration_efficiency():
    battery_usage_dir = f'results_{algorithm_dir}/battery_usage/{batch}/csv_tables'
    accuracy_plots_dir = f'results_{algorithm_dir}/accuracy_plots/{batch}/csv_tables'
    output_dir = f'results_{algorithm_dir}/battery_efficiency/{batch}/csv_tables'

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Dictionary to store coverage data from accuracy plots
    coverage_data = {}

    # Read accuracy plots to extract coverage data
    for file in os.listdir(accuracy_plots_dir):
        if file.endswith('.csv'):
            parts = file.split('_')
            map_name = parts[1]
            if parts[2] == 'tilted':
                map_name += '_tilted'
            noise = float(parts[-4])
            spawn_time = int(parts[-1].replace('.csv', ''))

            filepath = os.path.join(accuracy_plots_dir, file)
            data = pd.read_csv(filepath)

            for _, row in data.iterrows():
                n_agents = row['n_agents']
                coverage = row['coverage']
                key = (map_name, noise, spawn_time, n_agents)
                coverage_data[key] = coverage

    # Read traveled path data and calculate exploration efficiency
    for file in os.listdir(battery_usage_dir):
        if file.endswith('.csv'):
            parts = file.split('_')
            map_name = parts[2]
            if parts[3] == 'tilted':
                map_name += '_tilted'
            noise = float(parts[-4])
            spawn_time = int(parts[-1].replace('.csv', ''))

            filepath = os.path.join(battery_usage_dir, file)
            data = pd.read_csv(filepath)

            efficiency_data = []

            for _, row in data.iterrows():
                n_agents = row['n_agents']
                agent_int = int(n_agents.split('_')[0])
                battery_usage = row['mean'] * agent_int

                key = (map_name, noise, spawn_time, n_agents)
                coverage = coverage_data.get(key, None)

                if coverage is not None:
                    exploration_efficiency = 100 * coverage / battery_usage if battery_usage > 0 else 0
                    efficiency_data.append({
                        # 'map': map_name,
                        # 'noise': noise,
                        # 'spawn_time': spawn_time,
                        'n_agents': n_agents,
                        'coverage': coverage,
                        'battery_usage_sum': battery_usage,
                        'exploration_efficiency': exploration_efficiency
                    })

            # Save exploration efficiency data to CSV
            output_file = os.path.join(output_dir, f'efficiency_{map_name}_noise_{noise}_spawn_time_{spawn_time}.csv')
            pd.DataFrame(efficiency_data).to_csv(output_file, index=False)
            print(f"Saved exploration efficiency data for {map_name}, noise: {noise}, spawn_time: {spawn_time} to {output_file}")

    print(f"Battery exploration efficiency data saved to {output_dir}")

def average_efficiency_over_all_factors_per_agents(metric):
    input_dir = f'results_{algorithm_dir}/{metric}/{batch}/csv_tables'
    output_file = f'results_{algorithm_dir}/{metric}/{batch}/averaged_efficiency_per_agents.csv'

    if not os.path.exists(input_dir):
        print(f"Input directory {input_dir} does not exist.")
        return

    aggregated_data = []

    # Read all efficiency files
    for file in os.listdir(input_dir):
        if file.endswith('.csv'):
            filepath = os.path.join(input_dir, file)
            data = pd.read_csv(filepath)
            aggregated_data.append(data)

    if not aggregated_data:
        print("No efficiency files found.")
        return

    # Concatenate all data
    combined_df = pd.concat(aggregated_data, ignore_index=True)

    # Group by 'n_agents' and calculate mean and sample standard deviation
    grouped = combined_df.groupby('n_agents')
    averaged_data = grouped.mean().reset_index()
    std_dev_data = grouped.std().reset_index()

    # Merge mean and standard deviation into a single DataFrame
    for column in combined_df.columns:
        if column not in ['n_agents']:
            std_dev_column = std_dev_data[column].rename(f"std_dev_{column}")
            averaged_data = pd.concat([averaged_data, std_dev_column], axis=1)
    
    #sort based on n_agents
    averaged_data['n_agents_numeric'] = averaged_data['n_agents'].apply(lambda x: int(x.split('_')[0]))
    averaged_data = averaged_data.sort_values(by=['n_agents_numeric']).drop(columns=['n_agents_numeric'])

    # Save the averaged data to a CSV file
    averaged_data.to_csv(output_file, index=False)
    print(f"Averaged {metric} data saved to {output_file}")


# def average_efficiency_over_all_factors_per_noise():
#     input_dir = f'results_{algorithm_dir}/traveled_efficiency/{batch}/csv_tables'
#     output_file = f'results_{algorithm_dir}/traveled_efficiency/{batch}/averaged_efficiency_per_noise.csv'

#     if not os.path.exists(input_dir):
#         print(f"Input directory {input_dir} does not exist.")
#         return

#     aggregated_data = []

#     # Read all efficiency files
#     for file in os.listdir(input_dir):
#         if file.endswith('.csv'):
#             filepath = os.path.join(input_dir, file)
#             data = pd.read_csv(filepath)
#             aggregated_data.append(data)

#     if not aggregated_data:
#         print("No efficiency files found.")
#         return

#     # Concatenate all data
#     combined_df = pd.concat(aggregated_data, ignore_index=True)

#     # Group by 'n_agents' and calculate mean and sample standard deviation
#     grouped = combined_df.groupby('noise')
#     averaged_data = grouped.mean().reset_index()
#     std_dev_data = grouped.std().reset_index()

#     # Merge mean and standard deviation into a single DataFrame
#     for column in combined_df.columns:
#         if column not in ['noise']:
#             std_dev_column = std_dev_data[column].rename(f"std_dev_{column}")
#             averaged_data = pd.concat([averaged_data, std_dev_column], axis=1)
    
#     #sort based on n_agents
#     averaged_data = averaged_data.sort_values(by=['noise']).drop(columns=['noise'])

#     # Save the averaged data to a CSV file
#     averaged_data.to_csv(output_file, index=False)
#     print(f"Averaged efficiency data saved to {output_file}")


def average_battery_efficiency_over_all_factors():
    input_dir = f'results_{algorithm_dir}/battery_efficiency/{batch}/csv_tables'
    output_file = f'results_{algorithm_dir}/battery_efficiency/{batch}/averaged_efficiency.csv'

    if not os.path.exists(input_dir):
        print(f"Input directory {input_dir} does not exist.")
        return

    aggregated_data = []

    # Read all efficiency files
    for file in os.listdir(input_dir):
        if file.endswith('.csv'):
            filepath = os.path.join(input_dir, file)
            data = pd.read_csv(filepath)
            aggregated_data.append(data)

    if not aggregated_data:
        print("No efficiency files found.")
        return

    # Concatenate all data
    combined_df = pd.concat(aggregated_data, ignore_index=True)

    # Group by 'n_agents' and calculate mean and sample standard deviation
    grouped = combined_df.groupby('n_agents')
    averaged_data = grouped.mean().reset_index()
    std_dev_data = grouped.std().reset_index()

    # Merge mean and standard deviation into a single DataFrame
    for column in combined_df.columns:
        if column not in ['n_agents']:
            std_dev_column = std_dev_data[column].rename(f"std_dev_{column}")
            averaged_data = pd.concat([averaged_data, std_dev_column], axis=1)
    
    #sort based on n_agents
    averaged_data['n_agents_numeric'] = averaged_data['n_agents'].apply(lambda x: int(x.split('_')[0]))
    averaged_data = averaged_data.sort_values(by=['n_agents_numeric']).drop(columns=['n_agents_numeric'])

    # Save the averaged data to a CSV file
    averaged_data.to_csv(output_file, index=False)
    print(f"Averaged battery efficiency data saved to {output_file}")
        
        
def aggregate_metrics_per_env_map_per_noise(metric):
    results_dir = f'results_{algorithm_dir}'
    aggregated_data = {}

    for metric_type in [metric]:
        metric_dir = os.path.join(results_dir, metric_type, f'{batch}', 'csv_tables')
        if not os.path.exists(metric_dir):
            continue

        for file in os.listdir(metric_dir):
            if not file.endswith('.csv'):
                continue

            # Extract env_map from filename
            parts = file.split('_')
            metric_name = parts[0]  # e.g., "ACP" or "CP(T_end)"
            # if metric_name == 'CP(T':
            #     metric_name = 'CP(T_end)'
            # elif metric_name == 'AC(T':
            #     metric_name = 'AC(T_end)'
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
    rows = []
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
                rows.append(row)
    aggregated_df = pd.DataFrame(rows)
    #change any column names with 'Standard Deviation' to 'pooled standard deviation'
    aggregated_df.rename(columns=lambda x: x.replace('Standard Deviation', '$s_p$'), inplace=True)

    aggregated_df.to_csv(f'results_{algorithm_dir}/{metric_type}/{batch}/aggregated_metrics.csv', index=False)
    # Separate aggregated data for certainty and coverage

# def aggregate_metrics_per_noise_level():

def average_aggregated_metrics(metric):
    input_file = f'results_{algorithm_dir}/{metric}/{batch}/aggregated_metrics.csv'
    output_file = f'results_{algorithm_dir}/{metric}/{batch}/averaged_metrics.csv'
    
    # Read the aggregated metrics CSV
    aggregated_df = pd.read_csv(input_file)
    
    # Group by 'noise' and calculate the mean for each group
    grouped = aggregated_df.groupby('noise')

    # Create a new DataFrame to store the averaged results
    averaged_df = grouped.mean().reset_index()

    # Add standard deviation columns for each numeric column
    for column in aggregated_df.columns:
        if column not in ['env_map', 'n_agents', 'noise'] and '$s_p$' not in column:
            std_dev_column = grouped[column].std().reset_index(name=f"std dev {column}")
            averaged_df = pd.merge(averaged_df, std_dev_column, on='noise', how='left')

    #remove columns with '$s_p$'
    averaged_df = averaged_df.loc[:, ~averaged_df.columns.str.contains('\$s_p\$')]

    # Save the averaged metrics to a new CSV file
    averaged_df.to_csv(output_file, index=False)
    
    print(f"Averaged metrics saved to {output_file}")

def average_aggregated_metrics_per_map_duo_n_agents_for_noise(env_map, metric, noise):
    input_file = f'results_{algorithm_dir}/{metric}/{batch}/aggregated_metrics.csv'
    output_file = f'results_{algorithm_dir}/{metric}/{batch}/averaged_metrics_{env_map}+tilted_per_n_agents_noise_{noise}.csv'
    
    # Read the aggregated metrics CSV
    aggregated_df = pd.read_csv(input_file)
    # Filter the DataFrame for the specified noise level
    aggregated_df = aggregated_df[aggregated_df['noise'] == noise]

    allowed_maps = [env_map, env_map + '_tilted']
    # Filter the DataFrame for the specified env_map
    aggregated_df = aggregated_df[aggregated_df['env_map'].isin(allowed_maps)]
    
    # Group by 'n_agents' and 'env_map' and calculate the mean for each group
    grouped = aggregated_df.groupby(['n_agents'])

    # Create a new DataFrame to store the averaged results
    averaged_df = grouped.mean().reset_index()

    # Add standard deviation columns for each numeric column
    for column in aggregated_df.columns:
        if column not in ['env_map', 'n_agents', 'noise'] and '$s_p$' not in column:
            std_dev_column = grouped[column].std().reset_index(name=f"std dev {column}")
            averaged_df = pd.merge(averaged_df, std_dev_column, on='n_agents', how='left')

    #remove columns with '$s_p$'
    averaged_df = averaged_df.loc[:, ~averaged_df.columns.str.contains('\$s_p\$')]
    #remove noise column
    averaged_df = averaged_df.drop(columns=['noise'])

    #sort based on int of n_agents
    averaged_df['n_agents_numeric'] = averaged_df['n_agents'].apply(lambda x: int(x.split('_')[0]))
    averaged_df = averaged_df.sort_values(by=['n_agents_numeric']).drop(columns=['n_agents_numeric'])


    # Save the averaged metrics to a new CSV file
    averaged_df.to_csv(output_file, index=False)
    
    print(f"Averaged metrics saved to {output_file}")

def average_aggregated_metrics_f1():
    input_file = f'results_{algorithm_dir}/accuracy_plots/{batch}/aggregated_metrics.csv'
    output_file = f'results_{algorithm_dir}/accuracy_plots/{batch}/averaged_metrics_f1.csv'
    
    # Read the aggregated metrics CSV
    aggregated_df = pd.read_csv(input_file)
    
    #for each row, calculate f1 score with recall and precision
    aggregated_df['f1_score'] = 0
    for index, row in aggregated_df.iterrows():
        precision = row['precision']
        recall = row['recall']
        if precision + recall == 0:
            f1_score = 0
        else:
            f1_score = 2 * (precision * recall) / (precision + recall)
        aggregated_df.at[index, 'f1_score'] = f1_score

    # Group by 'noise' and calculate the mean for each group
    grouped = aggregated_df.groupby('noise')

    # Create a new DataFrame to store the averaged results
    averaged_df = grouped.mean().reset_index()

    # Add standard deviation columns for each numeric column
    for column in aggregated_df.columns:
        if column not in ['env_map', 'n_agents', 'noise'] and '$s_p$' not in column:
            std_dev_column = grouped[column].std().reset_index(name=f"std dev {column}")
            averaged_df = pd.merge(averaged_df, std_dev_column, on='noise', how='left')

    averaged_df['min f1_score'] = aggregated_df.groupby('noise')['f1_score'].min().reset_index(name='min f1_score')['min f1_score']
    averaged_df['max f1_score'] = aggregated_df.groupby('noise')['f1_score'].max().reset_index(name='max f1_score')['max f1_score']
    #remove columns with '$s_p$'
    averaged_df = averaged_df.loc[:, ~averaged_df.columns.str.contains('\$s_p\$')]
    
    #remove columns with 'precision' and 'recall' and 'coverage'
    averaged_df = averaged_df.loc[:, ~averaged_df.columns.str.contains('precision|recall|coverage')]


    # Save the averaged metrics to a new CSV file
    averaged_df.to_csv(output_file, index=False)
    
    print(f"Averaged metrics saved to {output_file}")

def average_aggregated_metrics_cpm():
    input_file = f'results_{algorithm_dir}/accuracy_plots/{batch}/aggregated_metrics.csv'
    output_file = f'results_{algorithm_dir}/accuracy_plots/{batch}/averaged_metrics_cp_m.csv'
    
    # Read the aggregated metrics CSV
    aggregated_df = pd.read_csv(input_file)
    
    # Group by 'noise' and calculate the mean for each group
    grouped = aggregated_df.groupby('noise')

    # Create a new DataFrame to store the averaged results
    averaged_df = grouped.mean().reset_index()

    # Add standard deviation columns for each numeric column
    for column in aggregated_df.columns:
        if column not in ['env_map', 'n_agents', 'noise'] and '$s_p$' not in column:
            std_dev_column = grouped[column].std().reset_index(name=f"std dev {column}")
            averaged_df = pd.merge(averaged_df, std_dev_column, on='noise', how='left')

    #remove columns with '$s_p$'
    averaged_df = averaged_df.loc[:, ~averaged_df.columns.str.contains('\$s_p\$')]
    
    #remove columns with 'precision' and 'recall'
    averaged_df = averaged_df.loc[:, ~averaged_df.columns.str.contains('precision|recall')]


    # Save the averaged metrics to a new CSV file
    averaged_df.to_csv(output_file, index=False)
    
    print(f"Averaged metrics saved to {output_file}")

def extract_aggregated_metrics_per_n_agents():
    input_file = f'results_{algorithm_dir}/accuracy_plots/{batch}/aggregated_metrics.csv'
    output_dir = f'results_{algorithm_dir}/accuracy_plots/{batch}/per_n_agents'
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Read the aggregated metrics CSV
    aggregated_df = pd.read_csv(input_file)
    
    # Group by 'n_agents' and save each group to a separate CSV
    grouped = aggregated_df.groupby('n_agents')
    min_recall = grouped['recall'].min()
    max_recall = grouped['recall'].max()
    print("min recall:", min_recall)
    print("max recall:", max_recall)
    min_precision = grouped['precision'].min()
    max_precision = grouped['precision'].max()
    print("min precision:", min_precision)
    print("max precision:", max_precision)
    min_coverage = grouped['coverage'].min()
    max_coverage = grouped['coverage'].max()
    print("min coverage:", min_coverage)
    print("max coverage:", max_coverage)
    for n_agents, group in grouped:
        grouped_by_map = group.groupby('env_map')
        for map_name, map_group in grouped_by_map:
        #exclude 'n_agents' and 'env_map' columns as they will be the same 
            map_group = map_group.drop(columns=['n_agents', 'env_map'])    

            #sort by noise
            map_group = map_group.sort_values(by=['noise'])

            # Save each map group to a separate CSV
            map_output_file = os.path.join(output_dir, f'aggregated_metrics_{n_agents}_{map_name}.csv')
            map_group.to_csv(map_output_file, index=False)
        # output_file = os.path.join(output_dir, f'aggregated_metrics_{n_agents}_agents.csv')
        # group.to_csv(output_file, index=False)

def extract_aggregated_metrics_per_n_agents_per_map_duo():
    input_file = f'results_{algorithm_dir}/accuracy_plots/{batch}/aggregated_metrics.csv'
    output_dir = f'results_{algorithm_dir}/accuracy_plots/{batch}/per_n_agents'
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Read the aggregated metrics CSV
    aggregated_df = pd.read_csv(input_file)
    
    # Group by 'n_agents' and save each group to a separate CSV
    grouped = aggregated_df.groupby('n_agents')
    # min_recall = grouped['recall'].min()
    # max_recall = grouped['recall'].max()
    # print("min recall:", min_recall)
    # print("max recall:", max_recall)
    # min_precision = grouped['precision'].min()
    # max_precision = grouped['precision'].max()
    # print("min precision:", min_precision)
    # print("max precision:", max_precision)
    # min_coverage = grouped['coverage'].min()
    # max_coverage = grouped['coverage'].max()
    # print("min coverage:", min_coverage)
    # print("max coverage:", max_coverage)
    for n_agents, group in grouped:
        grouped_by_map = group.groupby(group['env_map'].str.replace('_tilted', '', regex=False))

        for map_name, map_group in grouped_by_map:
            #average rows with the same value for 'noise'
            map_group = map_group.groupby('noise', as_index=False).agg(['mean', 'std']).reset_index()
            # Flatten the multi-level columns
            map_group.columns = ['_'.join(col).strip('_') if isinstance(col, tuple) else col for col in map_group.columns]

            # #exclude 'n_agents' and 'env_map' columns as they will be the same 
            # map_group = map_group.drop(columns=['n_agents', 'env_map'])

            #drop any columns with '$s_p$'
            map_group = map_group.loc[:, ~map_group.columns.str.contains('\$s_p\$')]

            #sort by noise
            map_group = map_group.sort_values(by=['noise'])

            # Save each map group to a separate CSV
            map_output_file = os.path.join(output_dir, f'aggregated_metrics_{n_agents}_{map_name}+tilted.csv')
            map_group.to_csv(map_output_file, index=False)
        # output_file = os.path.join(output_dir, f'aggregated_metrics_{n_agents}_agents.csv')
        # group.to_csv(output_file, index=False)
        

configs = []

#get all files in 'implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl'
for file in os.listdir(f'implementation_and_examples/agent_implementation/configs/{batch}'):
    config_name = file.split('.')[0]
    configs.append(config_name)    

usb_drive = f'/media/hugo/Thesis_Data/{algorithm_dir}/'
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
if not os.path.exists(f'results_{algorithm_dir}/accuracy_plots'):
    os.makedirs(f'results_{algorithm_dir}/accuracy_plots')
if not os.path.exists(f'results_{algorithm_dir}/accuracy_plots/' + batch):
    os.makedirs(f'results_{algorithm_dir}/accuracy_plots/' + batch)

completed_experiments, categories_and_values = get_values_for_each_category(configs)   
# check_if_all_required_experiments_done(completed_experiments, categories_and_values)
# plot_certainty_with_different_configs(usb_drive, categories_and_values)
# efficiency_with_different_configs(usb_drive, categories_and_values)

# calculate_exploration_efficiency()
# average_efficiency_over_all_factors_per_agents()
# average_efficiency_over_all_factors_per_noise()
# calculate_battery_exploration_efficiency()
# average_battery_efficiency_over_all_factors()

# aggregate_metrics_per_env_map_per_noise('traveled_efficiency')
for env_map in ['house', 'office', 'museum']:
    average_aggregated_metrics_per_map_duo_n_agents_for_noise(env_map, 'traveled_efficiency', 1)

# average_aggregated_metrics('traveled_efficiency')

# average_aggregated_metrics_f1()
# average_aggregated_metrics_cpm()

# extract_aggregated_metrics_per_n_agents()
# extract_aggregated_metrics_per_n_agents_per_map_duo()