import os
import csv
import pandas as pd

from zipfile import ZipFile, Path

def check_certainty_csv(zip_file):
    with ZipFile(zip_file, "r") as zf:
        certainty_files = {}
        non_completed_experiments = 0
        n_completed_experiments = 0
        zip_path = Path(zf)

        all_files = zf.namelist()
        directories = set("/".join(f.split("/")[:-1]) for f in all_files if "/" in f)

        # # Check if the root directory exists
        # if not os.path.exists(root_dir):
        #     raise Exception('The root directory does not exist.')

        # Iterate through directories
        for root in directories:
            # Check if 'certainty.csv' exists in the current directory
            csv_path = f"{root}/certainty.csv"
            if csv_path in all_files:
                # Read the file's content
                with zf.open(csv_path) as f:
                    df = pd.read_csv(f)
                    if not df.empty:
                        # Extract the outermost directory name after root
                        outer_dir = root.split('/')[1]  # Adjust index if needed
                        if outer_dir not in certainty_files:
                            certainty_files[outer_dir] = []
                        # Store the path without 'certainty.csv'
                        certainty_files[outer_dir].append(root.replace('implementation_and_examples/', ''))
                        n_completed_experiments += 1
                        print('n_completed_experiments:', n_completed_experiments)
                    else:
                        print('The file is empty:', csv_path)
            else:
                # If the last dir doesn't start with 'S', skip
                if root.split('/')[-1][0] != 'S':
                    continue
                print('The file does not exist:', root)
                non_completed_experiments += 1
                # print('n_non_completed_experiments:', non_completed_experiments)

        print('Number of non-completed experiments:', non_completed_experiments)
        print('Number of completed experiments:', n_completed_experiments)
        return certainty_files

# Replace with the directory path you want to search
zip_file = '/media/hugo/main/experiment_results_laptop_13-03.zip'
certainty_files = check_certainty_csv(zip_file)

# export the list of certainty files to a csv file
for outer_dir, files in certainty_files.items():
    file_path = f'completed_experiments_{outer_dir}.csv'

    with open(file_path, 'a', newline='\n') as csvfile:
        writer = csv.writer(csvfile)
        for file in files:
            writer.writerow([file])

