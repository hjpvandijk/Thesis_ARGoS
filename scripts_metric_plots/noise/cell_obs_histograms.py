import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
import os

batch = "noise"
# batch = "noise_wallfollowing"

algorithm_dir = 'CLARE'
# algorithm_dir = 'CLARE_wallfollowing'


def plot_histogram(env_map, noise, num_agents, bin_width=1):
    combined_bins = []
    for spawn_time in [0, 100, 180]:
        print(f"Plotting histogram for {env_map} with noise {noise} and spawn time {spawn_time}")
        dir = f'{usb_drive}averaged_data/{batch}/cell_observations/'

        filename = f'{env_map}_spawn_time_{spawn_time}_noise_{noise}.csv'

        csv_path = f'{dir}{filename}'
        
        """
        Plots a histogram for the given number of agents based on the data in the CSV file.

        Parameters:
            csv_path (str): Path to the CSV file.
            num_agents (int): Number of agents to plot the histogram for.
            bin_width (int): Width of each bin in the histogram (default is 1).
        """
        # Load the CSV file
        data = pd.read_csv(csv_path)
        #remove row where the first column is 0
        data = data[data['observation_count'] != 0]
        #sort the data by the first column
        data = data.sort_values(data.columns[0])

        # Extract the relevant columns
        observations = data['observation_count']  # First column (number of observations)
        agent_column = f"{num_agents}_agents"

        if agent_column not in data.columns:
            raise ValueError(f"Column '{agent_column}' not found in the CSV file.")

        bins = []

        # Filter rows where the value in agent_column is greater than 0
        data = data[data[agent_column] > 0]
        # Get the maximum value in the first column
        max_observations = data['observation_count'].max()
        n_cells_at_max_observations = data[data['observation_count'] == max_observations][agent_column].sum()

        print(f"Max observations: {max_observations}: {n_cells_at_max_observations} cells")

        for i in range(0, int(max_observations) + bin_width, bin_width):
            bin_lower_excl = i
            bin_upper_incl = i + bin_width
            # Sum the cell counts for the current bin
            bin_sum = data[(data.iloc[:, 0] > bin_lower_excl) & (data.iloc[:, 0] <= bin_upper_incl)][agent_column].sum()
            bins.append(bin_sum)
        #combine bins
        combined_bins.append(bins)


    #average bins
    final_bins = []
    for i in range(max(len(combined_bins[0]), len(combined_bins[1]), len(combined_bins[2]))):
        bin_sum = 0
        for j in range(3):
            if i < len(combined_bins[j]):
                bin_sum += combined_bins[j][i]
        final_bins.append(bin_sum / 3)
    
    bins = final_bins
    
    removed_sum = 0

    # Remove all bins after the last bin above 1
    for i in range(len(bins) - 1, -1, -1):
        if bins[i] > 10:
            break
        else:
            removed = bins.pop()
            removed_sum += removed

    # Print how much percentage of the total we are displaying
    total= (sum(bins) + removed_sum)

    


    print(f"Average otal cells: {total/3}")
    print(f"Showing {(1 - (removed_sum / total)) * 100:.2f}% of the total cells")

    # Plot the histogram
    cmap = plt.get_cmap('tab10')
    color = cmap(0)  # Use the first color from the colormap

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(
        range(len(bins)), 
        bins, 
        width=1, 
        # edgecolor='black', 
        alpha=0.7, 
        color=color
    )
    ax.set_xticks(range(len(bins)))
    ax.set_xticklabels(
        [f"{i}-{i + bin_width}" for i in range(0, int(max_observations) + bin_width, bin_width)], 
        rotation=45, 
        fontsize=12
    )
    ax.set_xlabel("Number of Observations", fontsize=14)
    ax.set_ylabel("Number of Cells", fontsize=14)
    ax.tick_params(axis='y', labelsize=12)
    # ax.grid(axis='y', linestyle='--', alpha=0.7)

    # Save the plot
    plot_filename = f"{env_map}_spawn_time_{spawn_time}_noise_{noise}_histogram.png"
    # plt.savefig(f"{usb_drive}averaged_data/{batch}/cell_observations/{plot_filename}", dpi=300)

    plt.tight_layout()

    # plt.show()

def plot_histogram_comp(env_map, noise, num_agents, bin_width=1):
    usb_pre = f'/media/hugo/Thesis_Data/'
    algorithm_paths = {
        'BICLARE-PP': f'{usb_pre}CLARE/averaged_data/{batch}/cell_observations/',
        'BICLARE-WF': f'{usb_pre}CLARE_wallfollowing/averaged_data/{batch}_wallfollowing/cell_observations/',
        'FSP-QT': f'{usb_pre}Base_QT/averaged_data/{batch}_QT/cell_observations/',
        'FSP-G': f'{usb_pre}Base_cohesion_alignment/averaged_data/{batch}_cohesionalignment/cell_observations/',
        'FSP': f'{usb_pre}Base/averaged_data/{batch}/cell_observations/'
    }
    # Use colors from the tab10 colormap
    cmap = plt.get_cmap('tab10')
    algorithm_colors = [cmap(i) for i in range(len(algorithm_paths))]
    combined_bins = {alg: [] for alg in algorithm_paths.keys()}

    for spawn_time in [0, 100, 180]:
        for alg, path in algorithm_paths.items():
            file_path = f'{path}{env_map}_spawn_time_{spawn_time}_noise_{noise}.csv'
            data = pd.read_csv(file_path)

            # Remove rows where the first column is 0
            data = data[data['observation_count'] != 0]
            # Sort the data by the first column
            data = data.sort_values(data.columns[0])

            # Extract the relevant columns
            agent_column = f"{num_agents}_agents"
            if agent_column not in data.columns:
                raise ValueError(f"Column '{agent_column}' not found in the CSV file.")

            # Filter rows where the value in agent_column is greater than 0
            data = data[data[agent_column] > 0]
            max_observations = data['observation_count'].max()

            bins = []
            for i in range(0, int(max_observations) + bin_width, bin_width):
                bin_lower_excl = i
                bin_upper_incl = i + bin_width
                bin_sum = data[(data['observation_count'] > bin_lower_excl) & 
                               (data['observation_count'] <= bin_upper_incl)][agent_column].sum()
                bins.append(bin_sum)
            combined_bins[alg].append(bins)

    # Average bins for each algorithm
    final_bins = {alg: [] for alg in algorithm_paths.keys()}
    for alg, bins_list in combined_bins.items():
        max_len = max(len(bins) for bins in bins_list)
        for i in range(max_len):
            bin_sum = 0
            count = 0
            for bins in bins_list:
                if i < len(bins):
                    bin_sum += bins[i]
                    count += 1
            final_bins[alg].append(bin_sum / count if count > 0 else 0)

    # Remove all bins after the last bin above 10
    removed_sum = {alg: 0 for alg in algorithm_paths.keys()}
    for alg, bins in final_bins.items():
        for i in range(len(bins) - 1, -1, -1):
            if bins[i] > 10:
                break
            else:
                removed = bins.pop()
                removed_sum[alg] += removed
    # Print how much percentage of the total we are displaying
    total = {alg: (sum(bins) + removed_sum[alg]) for alg, bins in final_bins.items()}
    for alg in final_bins.keys():
        print(f"Average total cells for {alg}: {total[alg] / 3}")
        print(f"Showing {(1 - (removed_sum[alg] / total[alg])) * 100:.2f}% of the total cells")

    # Plot the histogram
    fig, ax = plt.subplots(figsize=(12, 8))
    bar_width = 0.8/len(algorithm_paths)
    max_len = max(len(bins) for bins in final_bins.values())
    x_indexes = np.arange(max_len)
    for i, (alg, bins) in enumerate(final_bins.items()):
        padded_bins = bins + [0] * (max_len - len(bins))  # Pad bins with zeros to match max length
        ax.bar(
            x_indexes + i * bar_width,
            padded_bins,
            width=bar_width,
            label=alg,
            color=algorithm_colors[i],
            alpha=0.7
        )

    # Plot the curve that the bins follow
    for i, (alg, bins) in enumerate(final_bins.items()):
        padded_bins = bins + [0] * (max_len - len(bins))  # Pad bins with zeros to match max length
        ax.plot(
            x_indexes + i * bar_width + bar_width / 2,
            padded_bins,
            label=f"{alg} (curve)",
            color=algorithm_colors[i],
            alpha=0.9,
            linestyle='--'
        )

    longest_bins = max(len(bins) for bins in final_bins.values())
    # Customize the plot
    ax.set_xticks(x_indexes + bar_width * (len(algorithm_paths) - 1) / 2)
    ax.set_xticklabels(
        [f"{i+1}-{i+1 + bin_width}" for i in range(0, longest_bins * bin_width, bin_width)],
        rotation=45,
        fontsize=16
    )
    ax.set_xlabel("Number of Observations", fontsize=25)
    ax.set_ylabel("Number of Cells", fontsize=25)
    ax.tick_params(axis='y', labelsize=22)
    handles, labels = ax.get_legend_handles_labels()
    filtered_handles_labels = [(h, l) for h, l in zip(handles, labels) if "curve" not in l]
    ax.legend(*zip(*filtered_handles_labels), fontsize=22, ncol=2)
    plt.tight_layout()

    if not os.path.exists(f"results_comp/map_obs/noise/"):
        os.makedirs(f"results_comp/map_obs/noise/")

    # Save the plot
    # plot_filename = f"{env_map}_{num_agents}_agents_noise_{noise}_comparison_histogram.png"
    # plt.savefig(f"results_comp/map_obs/noise/{plot_filename}", dpi=300)
    plot_filename = f"{env_map}_{num_agents}_agents_noise_{noise}_comparison_histogram.pdf"
    plt.savefig(f"results_comp/map_obs/noise/{plot_filename}", dpi=300, format='pdf', bbox_inches='tight')
    #close the plot
    plt.close(fig)
    # plt.show()

# Function to lighten an RGBA color
def lighten_color(color, amount=0.5):
    """
    Lightens the given color by mixing it with white.
    amount: 0 < amount <= 1, where 1 is fully white.
    """
    import matplotlib.colors as mc
    c = mc.to_rgb(color)
    c_white = np.array([1.0, 1.0, 1.0])
    return mc.to_rgba(c + (c_white - c) * amount)

usb_drive = f'/media/hugo/Thesis_Data/{algorithm_dir}/'

lighten_factor = 0.2


for env_map in ['museum_tilted']:
# for env_map in ['house', 'house_tilted', 'office', 'office_tilted', 'museum', 'museum_tilted']:
    # for n_agents in [2, 4, 6, 10, 15]:
    #     plot_histogram_comp(env_map, 1, n_agents, 50)
    plot_histogram_comp(env_map, 1, 6, 50)

# plt.show()