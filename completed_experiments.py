import os
import csv
import pandas as pd

def check_certainty_csv(root_dir):
    certainty_files = {}
    non_completed_experiments = 0
    n_completed_experiments = 0

    # Check if the root directory exists
    if not os.path.exists(root_dir):
        raise Exception('The root directory does not exist.')

    # Walk through all directories and subdirectories
    for root, dirs, files in os.walk(root_dir):
        # Check if 'certainty.csv' exists in the current directory
        if 'certainty.csv' in files:
            file_path = os.path.join(root, 'certainty.csv')
            # Check if the file is not empty
            if not pd.read_csv(file_path).empty:
                # Extract the most outer directory name after the root dir
                outer_dir = root.split('/')[2]
                # Initialize the list if the key does not exist
                if outer_dir not in certainty_files:
                    certainty_files[outer_dir] = []
                # Remove 'implementation_and_examples/' from the file path
                file_path = file_path.replace('implementation_and_examples/', '')
                # Remove 'certainty.csv' from the file path
                file_path = file_path.replace('/certainty.csv', '')
                certainty_files[outer_dir].append(file_path)
                n_completed_experiments += 1
            else:
                print('The file is empty:', file_path)
        else:
            # If the last dir doesn't start with 'S', skip
            if root.split('/')[-1][0] != 'S':
                continue
            print('The file does not exist:', root)
            non_completed_experiments += 1

    print('Number of non-completed experiments:', non_completed_experiments)
    print('Number of completed experiments:', n_completed_experiments)
    return certainty_files

# Replace with the directory path you want to search
root_directory = 'implementation_and_examples/experiment_results'
certainty_files = check_certainty_csv(root_directory)

# export the list of certainty files to a csv file
for outer_dir, files in certainty_files.items():
    file_path = f'completed_experiments_{outer_dir}.csv'

    with open(file_path, 'a', newline='\n') as csvfile:
        writer = csv.writer(csvfile)
        for file in files:
            writer.writerow([file])

