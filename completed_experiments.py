import os
import csv
import pandas as pd


def check_certainty_csv(root_dir):
    certainty_files = []
    non_completed_experiments = 0

    # Check if the root directory exists
    if not os.path.exists(root_dir):
        # print('The root directory does not exist.')
        #throw an exception
        Exception('The root directory does not exist.')

    # Walk through all directories and subdirectories
    for root, dirs, files in os.walk(root_dir):
        # print('Checking directory:', root)
        # Check if 'certainty.csv' exists in the current directory
        if 'certainty.csv' in files:
            file_path = os.path.join(root, 'certainty.csv')
            # Check if the file is not empty
            if pd.read_csv(file_path).empty == False:
                #remove 'implementation_and_examples/' from the file path
                file_path = file_path.replace('implementation_and_examples/', '')
                #remove 'certainty.csv' from the file path
                file_path = file_path.replace('/certainty.csv', '')
                certainty_files.append(file_path)
            else:
                print('The file is empty:', file_path)
        else:
            #if the last dir doesn't start with 'S', skip
            if root.split('/')[-1][0] != 'S':
                continue
            print('The file does not exist:', root)
            non_completed_experiments += 1

    print('Number of non-completed experiments:', non_completed_experiments)
    return certainty_files

# Replace with the directory path you want to search
root_directory = 'implementation_and_examples/experiment_results'
certainty_files = check_certainty_csv(root_directory)

# export the list of certainty files to a csv file
with open('completed_experiments.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for file in certainty_files:
        writer.writerow([file])
