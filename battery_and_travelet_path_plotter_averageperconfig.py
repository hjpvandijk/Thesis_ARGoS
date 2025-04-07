import pandas as pd
import matplotlib.pyplot as plt
import os
import math
from matplotlib.widgets import CheckButtons

subplots = False
ticks_per_second = 16

path = 'implementation_and_examples/experiment_results'

#n configs
n_directories = 4
n_completed_agents = 9

map_max = {
    'museum': 500,
    'office': 180,
    'house': 80
}


for i, map in enumerate(os.listdir(path)):
    print("Map: ", map)
    for j, config in enumerate(os.listdir(os.path.join(path, map))):

        combined = pd.DataFrame()
        
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                # Construct the path to the CSV file in the current directory
                csv_path_battery = os.path.join(path, map, config, spawn_time, agents, 'battery_usage.csv')
                csv_path_traveled_path = os.path.join(path, map, config, spawn_time, agents, 'traveled_path.csv')


                # Check if the CSV file exists
                if os.path.exists(csv_path_battery):
                    #if csv file empty, skip
                    # if os.path.getsize(csv_path_battery) == 0:
                    #     n_completed_agents -= 1
                    #     continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path_battery)

                    if 'battery_usage' not in combined.columns:
                        combined['agent_id'] = data['agent_id']
                        combined['battery_usage'] = 0
                        combined['traveled_path'] = 0

                    combined['battery_usage'] += data['battery_usage']


                    
                # Check if the CSV file exists
                if os.path.exists(csv_path_traveled_path):
                    #if csv file empty, skip
                    # if os.path.getsize(csv_path_traveled_path) == 0:
                    #     continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path_traveled_path)
                    combined['traveled_path'] += data['traveled_path']
        #get the average over all agents
        battery_usage_average = combined['battery_usage'].mean()
        traveled_path_average = combined['traveled_path'].mean()
        #print the data
        print("\tConfig: ", config)
        print("\t\tBattery usage average: ", battery_usage_average)
        print("\t\tTraveled path average: ", traveled_path_average)
    
        


    plt.show()
