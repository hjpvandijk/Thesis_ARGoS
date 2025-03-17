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
        # if config not in ['n_3_m_2_5_cellratio0_75_noise', 'n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5', 'n_3_m_2_5_cellratio0_75_noise_object_safety_0_3']:
        #     continue
        if config != 'n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5':
            continue


        # combined = pd.DataFrame()
        dist_from_deployment = []
        
        for k, spawn_time in enumerate(os.listdir(os.path.join(path, map, config))):
            for l, agents in enumerate(os.listdir(os.path.join(path, map, config, spawn_time))):
                n_agents = int(agents.split('_')[0])
                if not (4 <= n_agents <= 8):
                    continue
                # Construct the path to the CSV file in the current directory
                csv_path_metrics = os.path.join(path, map, config, spawn_time, agents, 'distance_to_deployment_site.csv')


                # Check if the CSV file exists
                if os.path.exists(csv_path_metrics):
                    #if csv file empty, skip
                    # if os.path.getsize(csv_path_battery) == 0:
                    #     n_completed_agents -= 1
                    #     continue
                    # Read the CSV file
                    data = pd.read_csv(csv_path_metrics)
                    dist_from_deployment.append(data['distance_to_deployment_site'])
                else:
                    n_completed_agents -= 1

        # calculate average distance from deployment site
        total_average = 0
        for i in range(len(dist_from_deployment)):
            values = dist_from_deployment[i].values
            average = sum(values)/len(values)
            total_average += average
        total_average = total_average/len(dist_from_deployment)
        print("\tconfig: ", config)
        print("\t\tAVERAGE DIST FROM DEPLOYMENT SITE: ", total_average)
        # break
    print()

                    
     