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
n_completed_agents = 8-4+1

map_max = {
    'museum': 500,
    'office': 180,
    'house': 80
}

config_agent_agent_collisions = {
    'n_3_m_2_5_cellratio0_75_noise': [],
    'n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5': [],
    'n_3_m_2_5_cellratio0_75_noise_object_safety_0_3': []
}

config_agent_obstacle_collisions = {
    'n_3_m_2_5_cellratio0_75_noise': [],
    'n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5': [],
    'n_3_m_2_5_cellratio0_75_noise_object_safety_0_3': []
}



for i, map in enumerate(os.listdir(path)):
    print("Map: ", map)
    for j, config in enumerate(os.listdir(os.path.join(path, map))):
        if config not in ['n_3_m_2_5_cellratio0_75_noise', 'n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5', 'n_3_m_2_5_cellratio0_75_noise_object_safety_0_3']:
            continue


        # combined = pd.DataFrame()
        n_agent_agent_collisions = 0
        n_agent_obstacle_collisions = 0
        
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                n_agents = int(agents.split('_')[0])
                if not (4 <= n_agents <= 8):
                    continue
                # Construct the path to the CSV file in the current directory
                csv_path_metrics = os.path.join(path, map, config, spawn_time, agents, 'metrics.csv')


                # Check if the CSV file exists
                if os.path.exists(csv_path_metrics):
                    #if csv file empty, skip
                    # if os.path.getsize(csv_path_battery) == 0:
                    #     n_completed_agents -= 1
                    #     continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path_metrics)

                    # if 'n_agent_agent_collisions' not in combined.columns:
                    #     #initialize the columns
                    #     combined['n_agent_agent_collisions'] = 0
                    #     combined['n_agent_obstacle_collisions'] = 0
                    n_agent_agent_collisions += data['n_agent_agent_collisions'][0] / int(agents.split('_')[0])
                    n_agent_obstacle_collisions += data['n_agent_obstacle_collisions'][0] / int(agents.split('_')[0])
                else:
                    n_completed_agents -= 1
                    


                    
        #get the average over all agents
        n_agent_agent_collisions = n_agent_agent_collisions / n_completed_agents
        n_agent_obstacle_collisions = n_agent_obstacle_collisions / n_completed_agents

        config_agent_agent_collisions[config].append(n_agent_agent_collisions)
        config_agent_obstacle_collisions[config].append(n_agent_obstacle_collisions)

        #print the data
        print("\tConfig: ", config)
        print("\t\tn_completed_agents: ", n_completed_agents)
        print("\t\t\tAverage agent-agent collisions: ", n_agent_agent_collisions)
        print("\t\t\tAverage agent-obstacle collisions: ", n_agent_obstacle_collisions)

#get average collisions per config
for config in config_agent_agent_collisions:
    config_agent_agent_collisions[config] = sum(config_agent_agent_collisions[config]) / len(config_agent_agent_collisions[config])
    config_agent_obstacle_collisions[config] = sum(config_agent_obstacle_collisions[config]) / len(config_agent_obstacle_collisions[config])

print("Average agent-agent collisions per config: ", config_agent_agent_collisions)
print("Average agent-obstacle collisions per config: ", config_agent_obstacle_collisions)