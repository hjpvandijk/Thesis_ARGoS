import os
import csv
import pandas as pd
import json


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
                completed_unique[map].append(row[0])

overview = {}
#Create an overview per map, per configuration
for map, map_exp in completed_unique.items():
    if map not in overview:
        overview[map] = {}
    for exp in map_exp:
        split_exp = exp.split('/')
        config = split_exp[1]
        spawn_time = split_exp[2]
        agents = split_exp[3]
        seed = split_exp[4]

        if config not in overview[map]:
            overview[map][config] = {}
    
        if spawn_time not in overview[map][config]:
            overview[map][config][spawn_time] = {}

        if agents not in overview[map][config][spawn_time]:
            overview[map][config][spawn_time][agents] = []

        overview[map][config][spawn_time][agents].append(seed)
        overview[map][config][spawn_time][agents] = sorted(overview[map][config][spawn_time][agents])
        
   

# Sort the overview dictionary
sorted_overview = {
    map: {
        config: {
            spawn_time: {
                agents: sorted(seeds) for agents, seeds in sorted(spawn_times.items(), key=lambda x: int(x[0].split('_')[0]))
            } for spawn_time, spawn_times in sorted(configs.items())
        } for config, configs in sorted(map_configs.items())
    } for map, map_configs in sorted(overview.items())
}

# Export sorted overview to file in a readable format
with open('completed_in_zip/overview.json', 'w') as jsonfile:
    json.dump(sorted_overview, jsonfile, indent=4)
