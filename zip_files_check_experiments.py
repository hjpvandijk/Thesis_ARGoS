import os
import csv
import pandas as pd

from zipfile import ZipFile, Path

# n_completed_experiments = 0

def check_certainty_csv(zip_file):
    with ZipFile(zip_file, "r") as zf:
        certainty_files = {}
        non_completed_experiments = {}
        # non_completed_experiments = 0
        # n_completed_experiments = 0
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
                # print("csv_path: ", csv_path)
                # Read the file's content
                with zf.open(csv_path) as f:
                    df = pd.read_csv(f)
                        # Extract the outermost directory name after root
                    outer_dir = root.split('/')[1]  # Adjust index if needed
                    if outer_dir.startswith('end'):
                        outer_dir = root.split('/')[0]
                    else:
                        experiment_results_part = root.split('/')[0] + '/'
                        root = root.replace(experiment_results_part, '')
                    if not df.empty:
                        if outer_dir not in certainty_files:
                            certainty_files[outer_dir] = []
                        # Store the path without 'certainty.csv'
                        certainty_files[outer_dir].append(root.replace('implementation_and_examples/', ''))
                        # n_completed_experiments += 1
                        # print('[', zip_file, ']', 'n_completed_experiments:', n_completed_experiments)
                    else:
                        if outer_dir not in non_completed_experiments:
                            non_completed_experiments[outer_dir] = []
                        # print('The file is empty:', csv_path)
                        non_completed_experiments[outer_dir].append(root.replace('implementation_and_examples/', ''))
            else:
                # If the last dir doesn't start with 'S', skip
                if root.split('/')[-1][0] != 'S':
                    continue
                # print('The file does not exist:', csv_path)
                # non_completed_experiments += 1
                outer_dir = root.split('/')[1]  # Adjust index if needed
                if outer_dir.startswith('end'):
                    outer_dir = root.split('/')[0]
                else:
                    experiment_results_part = root.split('/')[0] + '/'
                    root = root.replace(experiment_results_part, '')

                if outer_dir not in non_completed_experiments:
                    non_completed_experiments[outer_dir] = []
                non_completed_experiments[outer_dir].append(root.replace('implementation_and_examples/', ''))

                # print('n_non_completed_experiments:', non_completed_experiments)

        # print('Number of non-completed experiments:', non_completed_experiments)
        # print('Number of completed experiments:', n_completed_experiments)
        return certainty_files, non_completed_experiments

completed_experiments = {}
completed_experiments_per_zip = {}
non_completed_experiments_all = {}
completed_total = 0
non_completed_total = 0
usb_drive = '/media/hugo/Philips_main/Base/'
#for all files in the usb drive
for file in os.listdir(usb_drive):
    if file.endswith(".zip"):
        if file =='experiment_results_afternoon_11-03.zip':
            continue
        zip_file = usb_drive + file
        print("checking zip file: ", zip_file)
        certainty_files, non_completed_experiments = check_certainty_csv(zip_file)
        for outer_dir, files in certainty_files.items():
            if outer_dir not in completed_experiments:
                completed_experiments[outer_dir] = []
            completed_experiments[outer_dir].extend(files)
            if zip_file not in completed_experiments_per_zip:
                completed_experiments_per_zip[zip_file] = {}
            if outer_dir not in completed_experiments_per_zip[zip_file]:
                completed_experiments_per_zip[zip_file][outer_dir] = []
            completed_experiments_per_zip[zip_file][outer_dir].extend(files)
            completed_total += len(files)
        for outer_dir, files in non_completed_experiments.items():
            if outer_dir not in non_completed_experiments_all:
                non_completed_experiments_all[outer_dir] = []
            non_completed_experiments_all[outer_dir].extend(files)
            non_completed_total += len(files)
    if non_completed_total > 0:
        continue

print("total completed experiments: ", completed_total)
print("total non completed experiments: ", non_completed_total)
completed_unique = {}
completed_unique_total = 0
for (map, map_exp) in completed_experiments.items():
    unique_exp = set(map_exp)
    completed_unique[map] = unique_exp
    completed_unique_total += len(unique_exp)

print("total completed experiments unique: ", completed_unique_total)

non_completed_unique = {}
non_completed_unique_total = 0
for (map, map_exp) in non_completed_experiments_all.items():
    unique_exp = set(map_exp)
    non_completed_unique[map] = unique_exp
    non_completed_unique_total += len(unique_exp)

n_non_completed_but_completed = 0
non_completed_but_completed = []
for (map, map_exp) in non_completed_unique.items():
    #check if in completed_unique
    if map in completed_unique:
        for exp in map_exp:
            if exp in completed_unique[map]:
                n_non_completed_but_completed += 1
                non_completed_but_completed.append(exp)

for exp in non_completed_but_completed:
    for map, map_exp in non_completed_unique.items():
        if exp in map_exp:
            map_exp.remove(exp)

for i in range (len(non_completed_unique.items())):
    #if the set is empty, remove it
    if len(list(non_completed_unique.values())[i]) == 0:
        del list(non_completed_unique.keys())[i]

print("total non completed but completed: ", n_non_completed_but_completed)
print("so actual non completed: ", non_completed_total - n_non_completed_but_completed)

#create 'completed_in_zip' directory
if not os.path.exists('completed_in_zip'):
    os.makedirs('completed_in_zip')

#empty the directory
for file in os.listdir('completed_in_zip'):
    #if it is a directory
    if os.path.isdir(f'completed_in_zip/{file}'):
        for f in os.listdir(f'completed_in_zip/{file}'):
            os.remove(f'completed_in_zip/{file}/{f}')
    else:
        os.remove(f'completed_in_zip/{file}')

# export the list of certainty files to a csv file
for outer_dir, files in completed_unique.items():
    file_path = f'completed_in_zip/completed_experiments_unique_{outer_dir}.csv'

    with open(file_path, 'w', newline='\n') as csvfile:
        writer = csv.writer(csvfile)
        for file in files:
            writer.writerow([file])

for outer_dir, files in non_completed_unique.items():
    file_path = f'completed_in_zip/non_completed_experiments_{outer_dir}.csv'

    with open(file_path, 'w', newline='\n') as csvfile:
        writer = csv.writer(csvfile)
        for file in files:
            writer.writerow([file])

if not os.path.exists('completed_in_zip/perzip'):
    os.makedirs('completed_in_zip/perzip')

for zip_file in completed_experiments_per_zip.keys():
    for outer_dir, files in completed_experiments_per_zip[zip_file].items():
        file_path = f'completed_in_zip/perzip/completed_experiments_unique_{outer_dir}_{zip_file.split("/")[-1]}.csv'

        with open(file_path, 'w', newline='\n') as csvfile:
            writer = csv.writer(csvfile)
            for file in files:
                writer.writerow([file])

