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
    #make all colors a bit lighter
    colors = [(r+0.2, g+0.2, b+0.2) for r, g, b in colors]    
    
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
    ax.set_xlabel('$f_e$', fontsize=16)
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

def barplots_f1_overall_comp_algorithms():
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_f1.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_f1.csv'
    file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/noise_QT/averaged_metrics_f1.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_f1.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/noise/averaged_metrics_f1.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE_QT = pd.read_csv(file_path_BASE_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]    
    colors = [lighten_color(c, lighten_factor) for c in colors]    

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
        bar_width = 0.8/len(algorithm_datas)
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
    ax.set_xticklabels(x, fontsize=22)
    ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=22)
    ax.set_xlabel('$f_e$', fontsize=25)
    ax.set_ylabel('F1 score (%)', fontsize=25)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=22, ncol=2)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    # output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_f1_barplot_overall.png'
    # plt.savefig(output_path)
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_f1_barplot_overall.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    
    #also make a lineplot
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metrics = ['f1_score']


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
        x_indexes = np.arange(len(x))

        color = colors[i]
        # Plot each algorithm's data
        for metric in metrics:
            metric_name = metric
            ax.plot(
                x, 
                data[metric], 
                label=f'{algorithm_names[i]}',
                color=color,
                marker='o',
                linewidth=2
            )
            ax.fill_between(
                x,
                data[metric] - data[f'std dev {metric}'],
                data[metric] + data[f'std dev {metric}'],
                color=color, 
                alpha=0.3
            )
    # Customize the plot
    ax.set_xticks(x)
    ax.set_xticklabels(x, fontsize=16)
    # ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$f_e$', fontsize=18)
    ax.set_ylabel('F1 score (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()
    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')
    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_f1_lineplot_overall.png'
    plt.savefig(output_path)
    # plt.show()


def barplots_cpm_overall_comp_algorithms():
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_cp_m.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_cp_m.csv'
    file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/noise_QT/averaged_metrics_cp_m.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_cp_m.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/noise/averaged_metrics_cp_m.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE_QT = pd.read_csv(file_path_BASE_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]    
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metric = 'coverage'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
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
    ax.set_ylim(0,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$f_e$', fontsize=18)
    ax.set_ylabel('$cp_m$ (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_barplot_overall.png'
    plt.savefig(output_path)
    # plt.show()
    #also make a lineplot
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metric = 'coverage'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
        x_indexes = np.arange(len(x))

        color = colors[i]
        # Plot each algorithm's data
        
        ax.plot(
            x, 
            data[metric], 
            label=f'{algorithm_names[i]}',
            color=color,
            marker='o',
            linewidth=2
        )
        ax.fill_between(
            x,
            data[metric] - data[f'std dev {metric}'],
            data[metric] + data[f'std dev {metric}'],
            color=color, 
            alpha=0.3
        )
    # Customize the plot
    ax.set_xticks(x)
    ax.set_xticklabels(x, fontsize=22)
    # ax.set_ylim(0,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=22)
    ax.set_xlabel('$f_e$', fontsize=25)
    ax.set_ylabel('$cp_m$ (%)', fontsize=25)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=22, ncol=2)
    plt.tight_layout()
    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')
    # Save and show the plot
    # output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_lineplot_overall.png'
    # plt.savefig(output_path)
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_lineplot_overall.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    # plt.show()


def barplots_cpm_per_agents_noise_0_comp_algorithms():
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_cp_m_per_agents_noise_0.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_cp_m_per_agents_noise_0.csv'
    file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/noise_QT/averaged_metrics_cp_m_per_agents_noise_0.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_cp_m_per_agents_noise_0.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/noise/averaged_metrics_cp_m_per_agents_noise_0.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE_QT = pd.read_csv(file_path_BASE_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]   
    colors = [lighten_color(c, lighten_factor) for c in colors]     
    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data['n_agents'] = data['n_agents'].str.extract('(\d+)_agents').astype(int)
        data.sort_values(by='n_agents', inplace=True)

        metric = 'coverage'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['n_agents']
        bar_width = 0.8/len(algorithm_datas)
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
    # ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$N_A$', fontsize=18)
    ax.set_ylabel('$cp_m$ (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_barplot_per_agents_noise_0.png'
    plt.savefig(output_path)
    
    #also make a line plot
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='n_agents', inplace=True)

        metric = 'coverage'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['n_agents']
        bar_width = 0.8/len(algorithm_datas)
        x_indexes = np.arange(len(x))

        color = colors[i]
        # Plot each algorithm's data
        
        ax.plot(
            x, 
            data[metric], 
            label=f'{algorithm_names[i]}',
            color=color,
            marker='o',
            linewidth=2
        )
        ax.fill_between(
            x,
            data[metric] - data[f'std dev {metric}'],
            data[metric] + data[f'std dev {metric}'],
            color=color, 
            alpha=0.3
        )
    # Customize the plot
    ax.set_xticks(x)
    ax.set_xticklabels(x, fontsize=16)
    # ax.set_ylim(50,110)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$N_A$', fontsize=18)
    ax.set_ylabel('$cp_m$ (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()
    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')
    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_lineplot_per_agents_noise_0.png'
    plt.savefig(output_path)


def barplots_efficiency_per_agents_noise_comp_algorithms(metric, noise):
    for env_map in ['house', 'office', 'museum']:
        # Load the CSV file
        file_path_BICLARE = f'results_CLARE/{metric}/noise/averaged_metrics_{env_map}+tilted_per_n_agents_noise_{noise}.csv' 
        file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/{metric}/noise_wallfollowing/averaged_metrics_{env_map}+tilted_per_n_agents_noise_{noise}.csv'
        file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/{metric}/noise_cohesionalignment/averaged_metrics_{env_map}+tilted_per_n_agents_noise_{noise}.csv'
        file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/{metric}/noise/averaged_metrics_{env_map}+tilted_per_n_agents_noise_{noise}.csv'
        data_BICLARE= pd.read_csv(file_path_BICLARE)
        data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
        data_BASE = pd.read_csv(file_path_BASE)
        data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
        algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE, data_BASE_NCA]
        algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-G', 'FSP']
        # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
        # algorithm_names = ['BICLARE', 'BICLARE_WF']
        n_colors = len(algorithm_datas)
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in range(n_colors)]   
        colors = [lighten_color(c, lighten_factor) for c in colors]     
        # Sort data by noise for each algorithm
        fig, ax = plt.subplots(figsize=(10, 6))
        for i, data in enumerate(algorithm_datas):
            #remove rows with noise 0.5 and 1.5
            # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
            data['n_agents'] = data['n_agents'].str.extract('(\d+)_agents').astype(int)
            data.sort_values(by='n_agents', inplace=True)

            metric_name = 'exploration_efficiency'


            # Plotting
            # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
            x = data['n_agents']
            bar_width = 0.8/len(algorithm_datas)
            x_indexes = np.arange(len(x))

            color = colors[i]
            # Plot each algorithm's data
            
            ax.bar(
                x_indexes + i * bar_width, 
                data[metric_name], 
                width=bar_width, 
                label=f'{algorithm_names[i]}',
                yerr=data[f'std dev {metric_name}'], 
                capsize=5, 
                color=color
            )

        #min is minimum value of the metric minus std dev
        # min_value = min([data[metric_name].min() for data in algorithm_datas])
        

        # Customize the plot
        ax.set_xticks(x_indexes + bar_width)
        ax.set_xticklabels(x, fontsize=16)
        # ax.set_ylim(min_value, max_value)
        ax.set_yticklabels(ax.get_yticks(), fontsize=16)
        ax.set_xlabel('$N_A$', fontsize=18)
        ax.set_ylabel('$E_p$', fontsize=18)
        # ax.set_title('Metric Accuracy vs Noise')
        ax.legend(fontsize=16)
        plt.tight_layout()

        if not os.path.exists(f'results_comp/{metric}/noise/barplots'):
            os.makedirs(f'results_comp/{metric}/noise/barplots')

        # Save and show the plot
        output_path = f'results_comp/{metric}/noise/barplots/map_{env_map}+tilted_accuracy_efficiency_barplot_per_agents_noise_0.png'
        plt.savefig(output_path)
    # plt.show()

def barplots_efficiency_overall_comp_algorithms():
    efficiency_dir = 'traveled_efficiency'
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/{efficiency_dir}/noise/averaged_metrics.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/{efficiency_dir}/noise_wallfollowing/averaged_metrics.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/{efficiency_dir}/noise_cohesionalignment/averaged_metrics.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/{efficiency_dir}/noise/averaged_metrics.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-G', 'FSP']
    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]    
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metric = 'exploration_efficiency'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
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
    ax.set_ylim(-30,150)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$f_e$', fontsize=18)
    ax.set_ylabel('$E_t$ (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/{efficiency_dir}/noise/barplots'):
        os.makedirs(f'results_comp/{efficiency_dir}/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/{efficiency_dir}/noise/barplots/efficiency_barplot_overall.png'
    plt.savefig(output_path)
    # plt.show()

def barplots_return_rate_overall_comp_algorithms():
    efficiency_dir = 'return_rate'
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/{efficiency_dir}/noise/averaged_metrics.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/{efficiency_dir}/noise_wallfollowing/averaged_metrics.csv'
    file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/{efficiency_dir}/noise_QT/averaged_metrics.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/{efficiency_dir}/noise_cohesionalignment/averaged_metrics.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/{efficiency_dir}/noise/averaged_metrics.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE_QT = pd.read_csv(file_path_BASE_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE', 'BICLARE_WF']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]    
    
    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metric = 'mean'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
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
    ax.set_ylim(0,1)
    ax.set_yticklabels([f'{ytick:.1f}' for ytick in ax.get_yticks()], fontsize=16)
    ax.set_xlabel('$f_e$', fontsize=18)
    ax.set_ylabel('Return rate', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=16)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/{efficiency_dir}/noise/barplots'):
        os.makedirs(f'results_comp/{efficiency_dir}/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/{efficiency_dir}/noise/barplots/return_rate_barplot_overall.png'
    plt.savefig(output_path)
    
    #also make a lineplot
    fig, ax = plt.subplots(figsize=(10, 6))
    for i, data in enumerate(algorithm_datas):
        #remove rows with noise 0.5 and 1.5
        # data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
        data.sort_values(by='noise', inplace=True)

        metric = 'mean'


        # Plotting
        # x = algorithm_datas[0]['noise']  # Assuming all algorithms have the same noise values
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
        x_indexes = np.arange(len(x))

        color = colors[i]
        # Plot each algorithm's data
        
        ax.plot(
            x, 
            data[metric], 
            label=f'{algorithm_names[i]}',
            color=color,
            marker='o',
            linewidth=2
        )
        ax.fill_between(
            x,
            data[metric] - data[f'std dev {metric}'],
            data[metric] + data[f'std dev {metric}'],
            color=color, 
            alpha=0.3
        )


    # Customize the plot
    ax.set_xticks(x)
    ax.set_xticklabels(x, fontsize=22)
    ax.set_ylim(0,1)
    ax.set_yticklabels([f'{ytick:.1f}' for ytick in ax.get_yticks()], fontsize=22)
    ax.set_xlabel('$f_e$', fontsize=25)
    ax.set_ylabel('Return rate', fontsize=25)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=22, ncol=2)
    plt.tight_layout()
    if not os.path.exists(f'results_comp/{efficiency_dir}/noise/barplots'):
        os.makedirs(f'results_comp/{efficiency_dir}/noise/barplots')
    # Save and show the plot
    # output_path = f'results_comp/{efficiency_dir}/noise/barplots/return_rate_lineplot_overall.png'
    # plt.savefig(output_path)

    output_path = f'results_comp/{efficiency_dir}/noise/barplots/return_rate_lineplot_overall.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")


    


def barplots_traveled_path_battery_overall_comp_algorithms():
    # Load the CSV files for both metrics
    path_dir = 'traveled_path'
    battery_dir = 'battery_usage'
    
    # File paths for traveled path
    file_path_BICLARE_path = f'results_CLARE/{path_dir}/noise/averaged_metrics.csv' 
    file_path_BICLARE_wallfollowing_path = f'results_CLARE_wallfollowing/{path_dir}/noise_wallfollowing/averaged_metrics.csv'
    file_path_BASE_QT_path = f'../Thesis_ARGoS2/results_Base_QT/{path_dir}/noise_QT/averaged_metrics.csv'
    file_path_BASE_path = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/{path_dir}/noise_cohesionalignment/averaged_metrics.csv'
    file_path_BASE_NCA_path = f'../Thesis_ARGoS2/results_Base/{path_dir}/noise/averaged_metrics.csv'
    
    # File paths for battery usage
    file_path_BICLARE_battery = f'results_CLARE/{battery_dir}/noise/averaged_metrics.csv' 
    file_path_BICLARE_wallfollowing_battery = f'results_CLARE_wallfollowing/{battery_dir}/noise_wallfollowing/averaged_metrics.csv'
    file_path_BASE_QT_battery = f'../Thesis_ARGoS2/results_Base_QT/{battery_dir}/noise_QT/averaged_metrics.csv'
    file_path_BASE_battery = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/{battery_dir}/noise_cohesionalignment/averaged_metrics.csv'
    file_path_BASE_NCA_battery = f'../Thesis_ARGoS2/results_Base/{battery_dir}/noise/averaged_metrics.csv'
    
    # Read data for traveled path
    data_BICLARE_path = pd.read_csv(file_path_BICLARE_path)
    data_BICLARE_wallfollowing_path = pd.read_csv(file_path_BICLARE_wallfollowing_path)
    data_BASE_QT_path = pd.read_csv(file_path_BASE_QT_path)
    data_BASE_path = pd.read_csv(file_path_BASE_path)
    data_BASE_NCA_path = pd.read_csv(file_path_BASE_NCA_path)
    
    # Read data for battery usage
    data_BICLARE_battery = pd.read_csv(file_path_BICLARE_battery)
    data_BICLARE_wallfollowing_battery = pd.read_csv(file_path_BICLARE_wallfollowing_battery)
    data_BASE_QT_battery = pd.read_csv(file_path_BASE_QT_battery)
    data_BASE_battery = pd.read_csv(file_path_BASE_battery)
    data_BASE_NCA_battery = pd.read_csv(file_path_BASE_NCA_battery)
    
    # Organize data
    path_datas = [data_BICLARE_path, data_BICLARE_wallfollowing_path, data_BASE_QT_path, data_BASE_path, data_BASE_NCA_path]
    battery_datas = [data_BICLARE_battery, data_BICLARE_wallfollowing_battery, data_BASE_QT_battery, data_BASE_battery, data_BASE_NCA_battery]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
    
    n_colors = len(algorithm_names)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]
    
    # For each noise value, create a separate figure
    noise_values = path_datas[0]['noise'].sort_values().unique()
    
    for noise in noise_values:
        # Create figure with two y-axes
        fig, ax1 = plt.subplots(figsize=(12, 8))
        ax2 = ax1.twinx()
        
        # Width settings
        n_algorithms = len(algorithm_names)
        group_width = 0.8
        bar_width = group_width / (n_algorithms * 2)  # Each algorithm has 2 bars (path & battery)
        
        # Plot for each algorithm
        for i, (path_data, battery_data) in enumerate(zip(path_datas, battery_datas)):
            # Filter data for current noise
            path_row = path_data[path_data['noise'] == noise]
            battery_row = battery_data[battery_data['noise'] == noise]
            
            if len(path_row) == 0 or len(battery_row) == 0:
                continue
                
            # Calculate positions
            x_pos = i * 2 * bar_width
            
            # Plot traveled path on left axis
            ax1.bar(
                x_pos, 
                path_row['mean'].values[0], 
                width=bar_width, 
                color=colors[i],
                yerr=path_row['std dev mean'].values[0],
                capsize=5,
                label=f"{algorithm_names[i]} (Path)"
            )
            
            # Plot battery usage on right axis
            ax2.bar(
                x_pos + bar_width, 
                battery_row['mean'].values[0], 
                width=bar_width, 
                color=lighten_color(colors[i], 0.7),
                yerr=battery_row['std dev mean'].values[0],
                capsize=5,
                label=f"{algorithm_names[i]} (Battery)",
                hatch='//'
            )
        
        # Set titles and labels
        # ax1.set_title(f'Traveled Path & Battery Usage at $f_e$ = {noise}', fontsize=16)
        ax1.set_xlabel('Algorithm', fontsize=16)
        ax1.set_ylabel('Traveled Path (m)', fontsize=16, color='darkblue')
        ax2.set_ylabel('Battery Usage (mAh)', fontsize=16, color='darkred')
        
        # Set x-ticks
        x_ticks = np.arange(0, n_algorithms * 2 * bar_width, 2 * bar_width) + bar_width/2
        ax1.set_xticks(x_ticks)
        ax1.set_xticklabels(algorithm_names, fontsize=14, rotation=30, ha='right')
        
        # Set colors for axes and ticks
        ax1.tick_params(axis='y', labelcolor='darkblue', labelsize=14)
        ax2.tick_params(axis='y', labelcolor='darkred', labelsize=14)
        
        # Create combined legend
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper center', 
                    bbox_to_anchor=(0.5, -0.15), ncol=3, fontsize=14)
        
        plt.tight_layout()
        
        # Save figure
        if not os.path.exists(f'results_comp/resource_usage/noise/barplots'):
            os.makedirs(f'results_comp/resource_usage/noise/barplots')
        
        output_path = f'results_comp/resource_usage/noise/barplots/resource_usage_fe_{noise}.png'
        plt.savefig(output_path, bbox_inches='tight')
    
    # Also create a single figure with all noise values
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Create separate figures for traveled path and battery usage

    # Figure for traveled path
    fig_path = plt.figure(figsize=(10, 6))
    ax_path = fig_path.add_subplot(111)
    
    # Process each algorithm for traveled path
    for i, path_data in enumerate(path_datas):
        path_data.sort_values(by='noise', inplace=True)
        
        # Plot traveled path
        ax_path.plot(path_data['noise'], path_data['mean'], marker='o', label=algorithm_names[i], color=colors[i])
        ax_path.fill_between(path_data['noise'], 
                            path_data['mean'] - path_data['std dev mean'], 
                            path_data['mean'] + path_data['std dev mean'], 
                            alpha=0.2, color=colors[i])
    
    # Customize path plot
    # ax_path.set_title('Traveled Path vs Noise', fontsize=16)
    ax_path.set_xlabel('$f_e$', fontsize=25)
    ax_path.set_ylabel('Traveled Path (m)', fontsize=25)
    ax_path.legend(fontsize=22, ncol=2)
    ax_path.grid(True, linestyle='--', alpha=0.7)
    ax_path.tick_params(axis='both', labelsize=22)
    
    plt.tight_layout()
    
    # Save the path figure
    # output_path_fig = f'results_comp/resource_usage/noise/barplots/traveled_path_summary.png'
    # plt.savefig(output_path_fig, bbox_inches='tight')
    output_path = f'results_comp/resource_usage/noise/barplots/traveled_path_summary.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    
    # Figure for battery usage
    fig_battery = plt.figure(figsize=(10, 6))
    ax_battery = fig_battery.add_subplot(111)
    
    # Process each algorithm for battery usage
    for i, battery_data in enumerate(battery_datas):
        battery_data.sort_values(by='noise', inplace=True)
        
        # Plot battery usage
        ax_battery.plot(battery_data['noise'], battery_data['mean'], marker='s', label=algorithm_names[i], color=colors[i])
        ax_battery.fill_between(battery_data['noise'], 
                            battery_data['mean'] - battery_data['std dev mean'], 
                            battery_data['mean'] + battery_data['std dev mean'], 
                            alpha=0.2, color=colors[i])
    
    # Customize battery plot
    # ax_battery.set_title('Battery Usage vs Noise', fontsize=16)
    ax_battery.set_xlabel('$f_e$', fontsize=25)
    ax_battery.set_ylabel('Estimated \n Battery Usage (mAh)', fontsize=25)
    ax_battery.legend(fontsize=22, ncol=2)
    ax_battery.grid(True, linestyle='--', alpha=0.7)
    ax_battery.tick_params(axis='both', labelsize=22)
    
    plt.tight_layout()
    
    # Save the battery figure
    # output_path_battery = f'results_comp/resource_usage/noise/barplots/battery_usage_summary.png'
    # plt.savefig(output_path_battery, bbox_inches='tight')
    output_path = f'results_comp/resource_usage/noise/barplots/battery_usage_summary.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    

def barplots_collisions_overall_comp_algorithms():
    efficiency_dir = 'collisions'
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/{efficiency_dir}/noise/averaged_metrics.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/{efficiency_dir}/noise_wallfollowing/averaged_metrics.csv'
    file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/{efficiency_dir}/noise_QT/averaged_metrics.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/{efficiency_dir}/noise_cohesionalignment/averaged_metrics.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/{efficiency_dir}/noise/averaged_metrics.csv'
    data_BICLARE= pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_BASE_QT = pd.read_csv(file_path_BASE_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    
    # Sort data by noise for each algorithm
    fig, ax = plt.subplots(figsize=(10, 6))

    # Create a DataFrame to store all data
    noise_values = algorithm_datas[0]['noise'].sort_values().unique()
    df = pd.DataFrame({'noise': noise_values})
    
    # For each algorithm, plot both obstacle and agent collisions
    for i, data in enumerate(algorithm_datas):
        data.sort_values(by='noise', inplace=True)
        
        x = data['noise']
        bar_width = 0.8/len(algorithm_datas)
        x_indexes = np.arange(len(x))
        
        # Plot obstacle collisions with regular color
        base_color = colors[i]
        # Make sure data is aligned with noise values
        for noise in noise_values:
            filtered_data = data[data['noise'] == noise]
            if not filtered_data.empty:
                idx = df[df['noise'] == noise].index[0]
                df.loc[idx, f'{algorithm_names[i]} (obstacle)'] = filtered_data['mean obstacle'].values[0]
                df.loc[idx, f'{algorithm_names[i]} (obstacle) std'] = filtered_data['std dev mean obstacle'].values[0]
                df.loc[idx, f'{algorithm_names[i]} (agent)'] = filtered_data['mean agent'].values[0]
                df.loc[idx, f'{algorithm_names[i]} (agent) std'] = filtered_data['std dev mean agent'].values[0]

        ax.bar(
            x_indexes + i * bar_width * 2, 
            data['mean obstacle'], 
            width=bar_width, 
            label=f'{algorithm_names[i]} (obstacle)',
            yerr=data['std dev mean obstacle'], 
            capsize=5, 
            color=base_color
        )
        
        # Plot agent collisions with lighter color
        light_color = lighten_color(base_color, 0.6)
        ax.bar(
            x_indexes + i * bar_width * 2 + bar_width, 
            data['mean agent'], 
            width=bar_width, 
            label=f'{algorithm_names[i]} (agent)',
            yerr=data['std dev mean agent'], 
            capsize=5, 
            color=light_color
        )

    # Customize the plot
    ax.set_xticks(x_indexes + bar_width * 4)
    ax.set_xticklabels(x, fontsize=16)
    # ax.set_ylim(0, 8)  # Adjust based on your data
    ax.set_yticklabels([f'{ytick:.1f}' for ytick in ax.get_yticks()], fontsize=16)
    ax.set_xlabel('$f_e$', fontsize=18)
    ax.set_ylabel('Collisions', fontsize=18)
    ax.legend(fontsize=12, ncol=2)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/{efficiency_dir}/noise/barplots'):
        os.makedirs(f'results_comp/{efficiency_dir}/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/{efficiency_dir}/noise/barplots/collisions_barplot_overall.png'
    plt.savefig(output_path)
    # plt.show()

    #export the df
    df.to_csv(f'results_comp/{efficiency_dir}/noise/barplots/collisions_overall.csv', index=False)

def plot_max_nodes_and_leaves():
    # Define the maps and file structure
    maps = ['house', 'office', 'museum']
    # n_agents = 15  # We can set the number of agents we want to plot
    spawn_time = 100
    noise = 1.5
    base_path = 'results_CLARE/max_nodes_and_leaves/noise/csv_tables'

    resolutions = {
        'house': 0.203125,
        'office': 0.1640625,
        'museum': 0.2421875
    }

    #root_sizes = max(actual_arena_width, actual_arena_height) + 1
    root_sizes = {
        'house': 13,
        'office': 21,
        'museum': 31
    }

    
    # Matrix cells constant for each map
    matrix_cells = {
        'house': (root_sizes['house'] **2) / (resolutions['house'] **2),
        'office': (root_sizes['office'] **2) / (resolutions['office'] **2),
        'museum': (root_sizes['museum'] **2) / (resolutions['museum'] **2)
    }

    agents = {
        'house': 10,
        'office': 10,
        'museum': 15,
    }
    
    # Lists to store data for each map
    map_data = {}
    for map_name in maps:
        file_path = f"{base_path}/max_nodes_and_leaves_{map_name}_noise_{noise}_spawn_time_{spawn_time}.csv"
        try:
            data = pd.read_csv(file_path)
            n_agents = agents[map_name]
            # Filter data for the specific number of agents
            row = data[data['n_agents'] == f"{n_agents}_agents"]
            if len(row) > 0:
                map_data[map_name] = {
                    'max_nodes': row['mean max nodes'].values[0],
                    'nodes_std': row['Sample Standard Deviation max nodes'].values[0],
                    'max_leaves': row['mean max leaves'].values[0],
                    'leaves_std': row['Standard Deviation max leaves'].values[0],
                    'matrix_cells': matrix_cells[map_name]
                }
            print("matrix_cells", matrix_cells[map_name])
        except FileNotFoundError:
            print(f"Warning: File not found: {file_path}")
    
    if not map_data:
        print("No data found for the specified maps and agents.")
        return
    
    # Set up the figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Width settings
    bar_width = 0.25
    index = np.arange(len(maps))
    
    # Colors for different bars
    colors = ['gray', 'skyblue', 'lightgreen']
    
    # Plot the data
    for i, map_name in enumerate(maps):
        if map_name in map_data:
            # Plot matrix cells
            ax.bar(
                index[i] - bar_width, 
                map_data[map_name]['matrix_cells'],
                width=bar_width,
                color=colors[0],
                label='Matrix cells' if i == 0 else None,
                hatch='///'
            )
            
            # Plot max nodes
            ax.bar(
                index[i], 
                map_data[map_name]['max_nodes'],
                width=bar_width,
                yerr=map_data[map_name]['nodes_std'],
                capsize=5,
                color=colors[1],
                label='Max nodes' if i == 0 else None
            )
            
            # Plot max leaves
            ax.bar(
                index[i] + bar_width, 
                map_data[map_name]['max_leaves'],
                width=bar_width,
                yerr=map_data[map_name]['leaves_std'],
                capsize=5,
                color=colors[2],
                label='Max leaves' if i == 0 else None,
                # hatch='///'
            )
    
    # Set the x-tick positions and labels
    ax.set_xticks(index)
    ax.set_xticklabels(maps, fontsize=16)
    
    # Add a legend
    ax.legend(fontsize=16, loc='upper right')
    
    # Add labels
    ax.set_xlabel('Map', fontsize=18)
    ax.set_ylabel('Count', fontsize=18)
    
    # Set title with number of agents
    # ax.set_title(f'Max Nodes and Leaves for {n_agents} agents', fontsize=18)
    
    # Ensure y-ticks are readable
    ax.tick_params(axis='y', labelsize=14)
    
    # Add grid lines for better readability
    # ax.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    
    # Save the plot
    output_dir = 'results_CLARE/max_nodes_and_leaves/noise/plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    output_path = f"{output_dir}/max_nodes_and_leaves_comparison.png"
    plt.savefig(output_path)
    plt.show()

    def perform_statistical_tests():
        import scipy.stats as stats
        
        # Test for F1 scores
        print("Performing statistical tests for F1 scores...")
        file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_f1.csv' 
        file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_f1.csv'
        file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/noise_QT/averaged_metrics_f1.csv'
        file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_f1.csv'
        file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/noise/averaged_metrics_f1.csv'
        
        data_BICLARE = pd.read_csv(file_path_BICLARE)
        data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
        data_BASE_QT = pd.read_csv(file_path_BASE_QT)
        data_BASE = pd.read_csv(file_path_BASE)
        data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
        
        algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_BASE_QT, data_BASE, data_BASE_NCA]
        algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']
        
        # Create output directory
        if not os.path.exists(f'results_comp/statistical_tests'):
            os.makedirs(f'results_comp/statistical_tests')
        
        # Initialize results dataframe
        results_f1 = pd.DataFrame(columns=['noise', 'algorithm1', 'algorithm2', 'p_value', 'significant', 'better_algorithm', 'mean_diff'])
        
        # For each noise value
        noise_values = data_BICLARE['noise'].unique()
        for noise in noise_values:
            for i in range(len(algorithm_datas)):
                for j in range(i+1, len(algorithm_datas)):
                    try:
                        # Get data for current noise level
                        data1 = algorithm_datas[i][algorithm_datas[i]['noise'] == noise]
                        data2 = algorithm_datas[j][algorithm_datas[j]['noise'] == noise]
                        
                        if len(data1) == 0 or len(data2) == 0:
                            continue
                            
                        # Extract means and standard deviations
                        mean1 = data1['f1_score'].values[0]
                        std1 = data1['std dev f1_score'].values[0]
                        mean2 = data2['f1_score'].values[0]
                        std2 = data2['std dev f1_score'].values[0]
                        
                        # Calculate t-statistic and p-value using Welch's t-test
                        # Assuming same number of samples for both algorithms
                        n1 = n2 = 4 * 6  # 4 runs per map, 6 maps
                        t_stat, p_val = stats.ttest_ind_from_stats(
                            mean1=mean1, std1=std1, nobs1=n1,
                            mean2=mean2, std2=std2, nobs2=n2,
                            equal_var=False  # Welch's t-test
                        )
                        
                        # Determine which algorithm is better
                        better_algo = algorithm_names[i] if mean1 > mean2 else algorithm_names[j]
                        mean_diff = abs(mean1 - mean2)
                        
                        # Add results to dataframe
                        results_f1 = results_f1.append({
                            'noise': noise,
                            'algorithm1': algorithm_names[i],
                            'algorithm2': algorithm_names[j],
                            'p_value': p_val,
                            'significant': p_val < 0.05,
                            'better_algorithm': better_algo,
                            'mean_diff': mean_diff
                        }, ignore_index=True)
                        
                    except Exception as e:
                        print(f"Error comparing {algorithm_names[i]} and {algorithm_names[j]} at noise {noise}: {e}")
        
        # Save F1 results
        results_f1.to_csv(f'results_comp/statistical_tests/f1_test_results.csv', index=False)
        
        # Print summary of significant differences for F1
        print("Summary of significant differences in F1 scores:")
        significant_results = results_f1[results_f1['significant'] == True]
        for _, row in significant_results.iterrows():
            print(f"Noise {row['noise']}: {row['better_algorithm']} is significantly better than {row['algorithm1'] if row['better_algorithm'] != row['algorithm1'] else row['algorithm2']} (p={row['p_value']:.4f}, diff={row['mean_diff']:.2f}%)")
        
        # Test for CPM scores
        print("\nPerforming statistical tests for CPM (coverage) scores...")
        file_path_BICLARE = f'results_CLARE/accuracy_plots/noise/averaged_metrics_cp_m.csv' 
        file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/noise_wallfollowing/averaged_metrics_cp_m.csv'
        file_path_BASE_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/noise_QT/averaged_metrics_cp_m.csv'
        file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/noise_cohesionalignment/averaged_metrics_cp_m.csv'
        file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/noise/averaged_metrics_cp_m.csv'
        
        data_BICLARE = pd.read_csv(file_path_BICLARE)


lighten_factor = 0.2

# # barplots_per_n_agents_per_map()
# # barplots_overall()
barplots_f1_overall_comp_algorithms()
barplots_cpm_overall_comp_algorithms()
# # barplots_efficiency_overall_comp_algorithms()
# # barplots_efficiency_per_agents_noise_comp_algorithms('traveled_efficiency', 1)
# barplots_cpm_per_agents_noise_0_comp_algorithms()
barplots_return_rate_overall_comp_algorithms()
# barplots_collisions_overall_comp_algorithms()
barplots_traveled_path_battery_overall_comp_algorithms()

# plot_max_nodes_and_leaves()

# plt.show()
