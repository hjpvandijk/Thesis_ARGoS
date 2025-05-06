import pandas as pd
import numpy as np
import os

import matplotlib.pyplot as plt

n_agents_list = [2,4,6,10,15]
env_map_list = ['house', 'house_tilted', 'office', 'office_tilted', 'museum', 'museum_tilted']
# env_map_list = ['office', 'office_tilted', 'museum', 'museum_tilted']

algorithm_dir = 'CLARE'

def barplots_per_n_agents_per_map():
    for n_agents in n_agents_list:
        for env_map in env_map_list:
            print(f'Processing {n_agents} agents in {env_map} map...')
            # Load the CSV file
            file_path = f'results_{algorithm_dir}/accuracy_plots/noise/per_n_agents/aggregated_metrics_{n_agents}_agents_{env_map}.csv'  # Update with the actual path
            data = pd.read_csv(file_path)
            #sort data by noise
            data = data.sort_values(by='noise')

            n_colors = 3
            # Set a pastel colormap
            cmap = plt.get_cmap('Pastel1', n_colors)
            # Set the colors for each metric
            colors = [cmap(i) for i in range(n_colors)]


            # # Calculate pooled standard deviation for each metric
            # def pooled_std(std_col, n=4):
            #     return np.sqrt(np.sum((n - 1) * (data[std_col] ** 2)) / (n * len(data) - len(data)))

            metrics = ['recall', 'precision', 'coverage']
            # pooled_stds = {metric: pooled_std(f'$sd_p$ {metric}') for metric in metrics}
            pooled_stds = {metric: data[f'$s_p$ {metric}'] for metric in metrics}

            # Plotting
            x = data['noise']
            bar_width = 0.2
            x_indexes = np.arange(len(x))

            fig, ax = plt.subplots(figsize=(10, 6))

            # Plot each metric
            for i, metric in enumerate(metrics):
                metric_name = metric
                color = colors[i]
                if metric == 'coverage':
                    metric_name = '$CP_m$'
                ax.bar(x_indexes + i * bar_width, data[metric], width=bar_width, label=metric_name, yerr=pooled_stds[metric], capsize=5, color=color)

            # Customize the plot
            ax.set_xticks(x_indexes + bar_width)
            ax.set_xticklabels(x)
            ax.set_ylim(50,110)
            ax.set_xlabel('Noise')
            ax.set_ylabel('%')
            # ax.set_title('Metric Accuracy vs Noise')
            ax.legend()
            plt.tight_layout()

            if not os.path.exists(f'results_{algorithm_dir}/accuracy_plots/noise/barplots'):
                os.makedirs(f'results_{algorithm_dir}/accuracy_plots/noise/barplots')

            # Save and show the plot
            output_path = f'results_{algorithm_dir}/accuracy_plots/noise/barplots/map_accuracy_barplot_{n_agents}_agents_{env_map}.png'
            plt.savefig(output_path)
            # plt.show()

def barplots_overall():
    # Load the CSV file
    file_path = f'results_{algorithm_dir}/accuracy_plots/noise/averaged_metrics.csv' 
    data = pd.read_csv(file_path)
    #sort data by noise
    data = data.sort_values(by='noise')


    metrics = ['recall', 'precision', 'coverage']
    pooled_stds = {metric: data[f'std dev {metric}'] for metric in metrics}

    n_colors = len(metrics)
    # Set a pastel colormap
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    # colors = [cmap(i+(i+1)*1) for i in range(n_colors)]
    
    
    # Plotting
    x = data['noise']
    bar_width = 0.2
    x_indexes = np.arange(len(x))

    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot each metric
    for i, metric in enumerate(metrics):
        metric_name = metric
        color = colors[i]
        if metric == 'coverage':
            metric_name = '$CP_m$'
        ax.bar(x_indexes + i * bar_width, data[metric], width=bar_width, label=metric_name, yerr=pooled_stds[metric], capsize=5, color=color)

    # Customize the plot
    ax.set_xticks(x_indexes + bar_width)
    ax.set_xticklabels(x, fontsize=16)
    ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$f_n$', fontsize=16)
    ax.set_ylabel('%', fontsize=16)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_{algorithm_dir}/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_{algorithm_dir}/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_{algorithm_dir}/accuracy_plots/noise/barplots/map_accuracy_barplot_overall.png'
    # plt.savefig(output_path)
    plt.show()

def barplots_f1_overall_comp_algorithms():
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_f1.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_f1.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_f1.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE = pd.read_csv(file_path_BASE)
    # algorithm_datas = [data_BICLARE, data_BASE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'Base_CA', 'BICLARE_WF']
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]    
    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metrics = ['f1_score']


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.2
        x_indexes = np.arange(len(x))

        color = colors[i]
        # Plot each algorithm's data
        for metric in metrics:
            metric_name = metric
            ax.bar(
                x_indexes + i * bar_width, 
                data[metric], 
                width=bar_width, 
                label=f'{algorithm_names[i]}',
                yerr=data[f'std dev {metric}'], 
                capsize=5, 
                color=color
            )

    # Customize the plot
    ax.set_xticks(x_indexes + bar_width)
    ax.set_xticklabels(x, fontsize=16)
    ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$f_n$', fontsize=16)
    ax.set_ylabel('F1 score (%)', fontsize=16)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_f1_barplot_overall.png'
    # plt.savefig(output_path)
    plt.show()

def barplots_cpm_overall_comp_algorithms():
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_cp_m.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_cp_m.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_cp_m.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE = pd.read_csv(file_path_BASE)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE]
    algorithm_names = ['BICLARE', 'BICLARE_WF', 'Base_CA']
    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]    
    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metric = 'coverage'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.2
        x_indexes = np.arange(len(x))

        color = colors[i]
        # Plot each algorithm's data
        
        ax.bar(
            x_indexes + i * bar_width, 
            data[metric], 
            width=bar_width, 
            label=f'{algorithm_names[i]}',
            yerr=data[f'std dev {metric}'], 
            capsize=5, 
            color=color
        )

    # Customize the plot
    ax.set_xticks(x_indexes + bar_width)
    ax.set_xticklabels(x, fontsize=16)
    ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$f_n$', fontsize=16)
    ax.set_ylabel('$cp_m$ (%)', fontsize=16)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_barplot_overall.png'
    # plt.savefig(output_path)
    plt.show()


# barplots_per_n_agents_per_map()
# barplots_overall()
barplots_f1_overall_comp_algorithms()
# barplots_cpm_overall_comp_algorithms()