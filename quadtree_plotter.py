import matplotlib.pyplot as plt
import csv



def plot_boxes(filename):
    min_x = -5
    max_x = 5
    min_y = -5
    max_y = 5
    # Read the file
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]

    # Group data by agent_id
    agents_data = {}
    for row in data:
        agent_id = row['agent_id']
        if agent_id not in agents_data:
            agents_data[agent_id] = []
        agents_data[agent_id].append(row)

    for agent_id, agent_data in agents_data.items():
        fig, ax = plt.subplots()

        for row in agent_data:
            box_size = float(row['box_size'])
            box_x = float(row['box_x']) - box_size / 2
            box_y = float(row['box_y']) - box_size / 2

            min_x = min(min_x, box_x)
            max_x = max(max_x, box_x + box_size)
            min_y = min(min_y, box_y)
            max_y = max(max_y, box_y + box_size)

            pheromone = float(row['pheromone'])

            # Calculate color based on pheromone value
            # color = (1 - pheromone, pheromone, 0)  # Red to Green gradient
            # if pheromone < 0.5:
            #     r = (0.5 - pheromone) * 2
            #     color = (r,0,0)
            # else :
            #     g = (pheromone - 0.5) * 2
            #     color = (0,g,0)

            # if pheromone < 0.3:
            #     color = (1,0,0)
            # else:
            #     color = (0,1,0)

            if pheromone < 0.3:
                color = (1,0,0)
            elif 0.3 <= pheromone < 0.5: 
                color = (0.5,0,0)
            elif 0.5 <= pheromone < 0.7:
                color = (0,0.5,0)
            else:
                color = (0,1,0)

            # Draw the rectangle
            rect = plt.Rectangle((box_x, box_y), box_size, box_size, color=color, alpha=1)
            #hovering over a box should show the pheromone value
            # ax.text(box_x + box_size / 2, box_y + box_size / 2, f'{pheromone:.2f}', ha='center', va='center')
            ax.add_patch(rect)

        ax.set_aspect('equal', 'box')
        
        # plt.xlim(min_x, max_x)
        # plt.ylim(min_y, max_y)
        plt.xlim(-15, 15)
        plt.ylim(-15, 15)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title(f'Boxes Plot for{filename} for Agent {agent_id}')

        plt.grid(True)
        # exit()

# Usage
for i in range(4):
    # if i+1 != 1:
    #     continue
    agent_id = "pipuck"+str(i + 1)
    # plot_boxes('implementation_and_examples/experiment_results/house_tilted/end_time_400_noise_0_wifi_range_15_message_loss_probability_0_frontier_search_radius_99999_evaporation_time_100_max_frontier_cells_99999_max_route_length_99999/spawn_time_100/15_agents/S1/quadtree_returning_'+agent_id+'.csv')
    # plot_boxes('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_1_wifi_range_15_message_loss_probability_0_1_frontier_search_radius_99999_evaporation_time_100_max_route_length_99999/spawn_time_0/6_agents/S1/quadtree_returning_'+agent_id+'.csv')
    # plot_boxes('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_1_wifi_range_15_message_loss_probability_0_1_frontier_search_radius_99999_evaporation_time_100_max_route_length_99999_1/spawn_time_0/6_agents/S1/quadtree_returning_'+agent_id+'.csv')
    # plot_boxes('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_1_wifi_range_15_message_loss_probability_0_1_frontier_search_radius_99999_max_frontier_regions_99999_evaporation_time_100_max_route_length_99999/spawn_time_100/4_agents/S1/quadtree_returning_'+agent_id+'.csv')
    # plot_boxes('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_1_wifi_range_15_message_loss_probability_0_1_frontier_search_radius_99999_max_frontier_regions_99999_evaporation_time_100_max_route_length_99999/spawn_time_100/4_agents/S1/quadtree_finished_exploring_'+agent_id+'.csv')
    plot_boxes('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_1_wifi_range_15_message_loss_probability_0_1_frontier_search_radius_99999_max_frontier_regions_99999_evaporation_time_100_max_route_length_99999/spawn_time_100/4_agents/S1/quadtree_map_relayed_'+agent_id+'.csv')
    
        # if config not in ['p_sensor_1', 'p_sensor_0_9', 'p_sensor_0_75', 'p_sensor_0_5']:
# plot_boxes('implementation_and_examples/experiment_results/house/AAVFIX_end_time_400_noise_1_wifi_range_15_message_loss_probability_0_1_frontier_search_radius_99999_evaporation_time_100_max_route_length_99999/spawn_time_0/15_agents/S1/quadtree_all_done.csv')

plt.show()
