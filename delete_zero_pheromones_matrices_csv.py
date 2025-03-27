import pandas as pd
import argparse
import os

def find_matrix_files(directory):
    matrix_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.startswith("coverage_matrix") or file.startswith("obstacle_matrix"):
                filepath = os.path.join(root, file)
                filter_csv(filepath)



def filter_csv(input_file):
    if "coverage_matrix" in input_file:
        column_name = "coverage_pheromone"
    elif "obstacle_matrix" in input_file:
        column_name = "obstacle_pheromone"
    else:
        print("Unknown file type.")
        return
    
    # Read the CSV file
    df = pd.read_csv(input_file)
    
    # Filter out rows where the specified column has value 0
    df_filtered = df[df[column_name] != 0]
    
    # Set output filename
    #output_file = input_file.replace(".csv", "_nonzero.csv")
    
    # Save the modified CSV
    df_filtered.to_csv(input_file, index=False)
    #print(f"Filtered CSV saved as {input_file}")

filename = "implementation_and_examples/experiment_results/office/BASE_AAVFIX_end_time_600_noise_0_wifi_range_99999_message_loss_probability_0_frontier_search_radius_5_max_frontier_regions_20_evaporation_time_100/spawn_time_0/2_agents/S5/coverage_matrix_all_done.csv"

output_file = "implementation_and_examples/experiment_results/office/BASE_AAVFIX_end_time_600_noise_0_wifi_range_99999_message_loss_probability_0_frontier_search_radius_5_max_frontier_regions_20_evaporation_time_100/spawn_time_0/2_agents/S5/coverage_matrix_all_done_nonzero.csv"

column_name = "coverage_pheromone"

directory = 'implementation_and_examples/experiment_results'

find_matrix_files(directory)
    
