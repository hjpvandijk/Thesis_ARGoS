import matplotlib.pyplot as plt
import csv

def plot_boxes(filename):
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

            pheromone = float(row['pheromone'])

            # Calculate color based on pheromone value
            color = (1 - pheromone, pheromone, 0)  # Red to Green gradient

            # Draw the rectangle
            rect = plt.Rectangle((box_x, box_y), box_size, box_size, color=color, alpha=1)
            ax.add_patch(rect)

        ax.set_aspect('equal', 'box')
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title(f'Boxes Plot for Agent {agent_id}')

        plt.grid(True)
        plt.show()
        exit()

# Usage
plot_boxes('implementation_and_examples/experiment_results/quadtree.csv')

