import pandas as pd
import numpy as np
import os

import matplotlib.pyplot as plt

n_agents_list = [2,4,6,10,15]
env_map_list = ['house', 'house_tilted', 'office', 'office_tilted', 'museum', 'museum_tilted']
# env_map_list = ['office', 'office_tilted', 'museum', 'museum_tilted']

batch = 'fsr_mfr_mrl'
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
            colors = [lighten_color(c, lighten_factor) for c in colors]    


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
    colors = [lighten_color(c, lighten_factor) for c in colors]    
    
    
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
    ax.set_xlabel('$f_n$', fontsize=18)
    ax.set_ylabel('%', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=18)
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
    algorithm_datas = [data_BICLARE, data_BASE, data_BICLARE_wallfollowing]
    algorithm_names = ['BICLARE', 'Base_CA', 'BICLARE_WF']
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
        data = data[(data['noise'] != 0.5) & (data['noise'] != 1.5)]
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
    ax.set_xlabel('$f_n$', fontsize=18)
    ax.set_ylabel('F1 score (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=18)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_f1_barplot_overall.png'
    # plt.savefig(output_path)
    plt.show()

def barplots_cpm_overall_comp_algorithms():
    # Load the CSV file
    file_path_BICLARE = f'results_CLARE/accuracy_plots/fsr_mfr_mrl/averaged_metrics_cp_m.csv' 
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/averaged_metrics_cp_m.csv'
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
    colors = [lighten_color(c, lighten_factor) for c in colors]    

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
    ax.set_xlabel('$f_n$', fontsize=18)
    ax.set_ylabel('$cp_m$ (%)', fontsize=18)
    # ax.set_title('Metric Accuracy vs Noise')
    ax.legend(fontsize=18)
    plt.tight_layout()

    if not os.path.exists(f'results_comp/accuracy_plots/noise/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/noise/barplots')

    # Save and show the plot
    output_path = f'results_comp/accuracy_plots/noise/barplots/map_accuracy_cpm_barplot_overall.png'
    # plt.savefig(output_path)
    plt.show()

def mfr_mrl_plots_f1_comp_algorithms(n_agents):
    # Load the CSV files for different algorithms
    file_path_BICLARE = f'results_CLARE/accuracy_plots/{batch}/mfr_mrl_averaged_metrics_f1_per_config.csv'
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/mfr_mrl_averaged_metrics_f1_per_config.csv'
    
    data_BICLARE = pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF']
    
    n_colors = len(algorithm_names)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Filter data for the specified n_agents
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define configurations to plot
    configs = [
        ('20', '30'),
        ('20', '$\\infty$'),
        ('$\\infty$', '30'),
        ('$\\infty$', '$\\infty$'),
    
    ]
    
    # Calculate bar width and positions
    n_configs = len(configs)
    n_algorithms = len(algorithm_names)
    group_width =  0.6  # Width for each mfr group
    bar_width = group_width / n_algorithms
    
    # Create DataFrame to store the results
    df = pd.DataFrame(columns=['mfr', 'mrl'] + algorithm_names + [f'{name} std dev' for name in algorithm_names])
    
    # Prepare data for plotting
    positions = []
    for i, (mfr, mrl) in enumerate(configs):
        config_pos = i * 1.5  # Space between configurations
        positions.append(config_pos)
        
        # Add to DataFrame
        row_data = {'mfr': mfr, 'mrl': mrl}
        
        for j, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
            # Skip BICLARE-WF for configs with mrl 30
            if algo_name == 'BICLARE-WF' and mrl == '30':
                continue
            # Filter data for the specified n_agents and noise 0
            filtered_data = data[(data['n_agents'] == f'{n_agents}_agents') & (data['noise'] == 0.0)]
            
            if not filtered_data.empty:
                # Get the column name for this configuration
                col_name = f'mfr {mfr} mrl {mrl} f1_score'
                std_col_name = f'std dev mfr {mfr} mrl {mrl} f1_score'
                
                # Get the values
                # The column names in the CSV have a different format for infinity symbol
                csv_col_name = col_name.replace('$\\infty$', '$\infty$')
                csv_std_col_name = std_col_name.replace('$\\infty$', '$\infty$')
                columns = filtered_data.columns

                #remove columns with 'min' 'max' 'median'
                columns = [col for col in columns if 'min' not in col and 'max' not in col and 'median' not in col]
                f1_score = filtered_data[csv_col_name].values[0]
                std_dev = filtered_data[csv_std_col_name].values[0]
                
                # Add to DataFrame
                row_data[algo_name] = f1_score
                row_data[f'{algo_name} std dev'] = std_dev
                
                # Plot the bar
                ax.bar(
                    config_pos + j * bar_width - group_width/2 + bar_width/2,
                    f1_score,
                    width=bar_width,
                    color=colors[j],
                    yerr=std_dev,
                    capsize=5,
                    label=algo_name if algo_name not in ax.get_legend_handles_labels()[1] else None
                )
        
        # Add the row to DataFrame
        df = df._append(row_data, ignore_index=True)
    
    # Add vertical separators between groups
    for i in range(len(configs) - 1):
        ax.axvline(positions[i] + 0.75, color='gray', linestyle='--', alpha=0.5)
    
    # Set x-axis labels
    ax.set_xticks(positions)
    ax.set_xticklabels([f'$N_f$={mfr}, $N_s$={mrl}' for mfr, mrl in configs], fontsize=22)
    
    # Set y-axis limits and labels
    ax.set_ylim(97, 99)
    # ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
    ax.set_yticklabels(ax.get_yticks(), fontsize=22)
    ax.set_ylabel('F1 Score', fontsize=25)
    # ax.set_title(f'F1 Score by MFR and MRL Configuration for {n_agents} Agents', fontsize=18)
    
    # Add legend
    ax.legend(fontsize=22, ncol=2, loc='lower center')
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')
    
    # Save the plot
    # output_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_mfr_mrl_{n_agents}_agents.png'
    # plt.savefig(output_path)
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_mfr_mrl_{n_agents}_agents.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    
    # Export the dataframe
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_mfr_mrl_{n_agents}_agents.csv'
    df.to_csv(output_path, index=False)
    
    # plt.show()

def mfr_mrl_plots_cpm_noise_0_comp_algorithms(n_agents):
    # Load the CSV files for different algorithms
    file_path_BICLARE = f'results_CLARE/accuracy_plots/{batch}/mfr_mrl_averaged_metrics_cp_m_per_agents_noise_0_per_config.csv'
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/mfr_mrl_averaged_metrics_cp_m_per_agents_noise_0_per_config.csv'
    
    data_BICLARE = pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF']
    
    n_colors = len(algorithm_names)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Filter data for the specified n_agents
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define configurations to plot
    configs = [
        ('20', '30'),
        ('20', '$\\infty$'),
        ('$\\infty$', '30'),
        ('$\\infty$', '$\\infty$'),
    
    ]
    
    
    # Calculate bar width and positions
    n_configs = len(configs)
    n_algorithms = len(algorithm_names)
    group_width = 0.6  # Width for each mfr group
    bar_width = group_width / n_algorithms
    
    # Create DataFrame to store the results
    df = pd.DataFrame(columns=['mfr', 'mrl'] + algorithm_names + [f'{name} std dev' for name in algorithm_names])
    
    # Prepare data for plotting
    positions = []
    for i, (mfr, mrl) in enumerate(configs):
        config_pos = i * 1.5  # Space between configurations
        positions.append(config_pos)
        
        # Add to DataFrame
        row_data = {'mfr': mfr, 'mrl': mrl}
        
        for j, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
            # Skip BICLARE-WF for configs with mrl 30
            if algo_name == 'BICLARE-WF' and mrl == '30':
                continue
            # Filter data for the specified n_agents and noise 0
            filtered_data = data[(data['n_agents'] == f'{n_agents}_agents') & (data['noise'] == 0.0)]
            
            if not filtered_data.empty:
                # Get the column name for this configuration
                col_name = f'mfr {mfr} mrl {mrl} coverage'
                std_col_name = f'std dev mfr {mfr} mrl {mrl} coverage'
                
                # Get the values
                # The column names in the CSV have a different format for infinity symbol
                csv_col_name = col_name.replace('$\\infty$', '$\infty$')
                csv_std_col_name = std_col_name.replace('$\\infty$', '$\infty$')
                
                # Remove columns with 'min' 'max' 'median'
                columns = filtered_data.columns
                columns = [col for col in columns if 'min' not in col and 'max' not in col and 'median' not in col]
                
                cp_m = filtered_data[csv_col_name].values[0]
                std_dev = filtered_data[csv_std_col_name].values[0]
                
                # Add to DataFrame
                row_data[algo_name] = cp_m
                row_data[f'{algo_name} std dev'] = std_dev
                
                # Plot the bar
                ax.bar(
                    config_pos + j * bar_width - group_width/2 + bar_width/2,
                    cp_m,
                    width=bar_width,
                    color=colors[j],
                    yerr=std_dev,
                    capsize=5,
                    label=algo_name if algo_name not in ax.get_legend_handles_labels()[1] else None
                )
        
        # Add the row to DataFrame
        df = df._append(row_data, ignore_index=True)
    
    # Add vertical separators between groups
    for i in range(len(configs) - 1):
        ax.axvline(positions[i] + 0.75, color='gray', linestyle='--', alpha=0.5)
    
    # Set x-axis labels
    ax.set_xticks(positions)
    ax.set_xticklabels([f'$N_f$={mfr}, $N_s$={mrl}' for mfr, mrl in configs], fontsize=25)
    
    # ax.set_ylim(25, 90)
    # Set y-axis limits and labels
    ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=25)
    ax.set_ylabel('$CP_m$', fontsize=25)
    # ax.set_title(f'Coverage by MFR and MRL Configuration for {n_agents} Agents', fontsize=18)
    
    # Add legend
    ax.legend(fontsize=22, ncol=2, loc='lower center')
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')
    
    # Save the plot
    # output_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_mfr_mrl_{n_agents}_agents.png'
    # plt.savefig(output_path)
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_mfr_mrl_{n_agents}_agents.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    
    # Export the dataframe
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_mfr_mrl_{n_agents}_agents.csv'
    df.to_csv(output_path, index=False)
    
    # plt.show()

def mfr_mrl_plots_ACP_noise_0_comp_algorithms(n_agents):
    # Load the CSV files for different algorithms
    file_path_BICLARE = f'results_CLARE/coverage_plots/{batch}/mfr_mrl_averaged_metrics_ACP_per_agents_noise_0_per_config.csv'
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/coverage_plots/fsr_mfr/mfr_mrl_averaged_metrics_ACP_per_agents_noise_0_per_config.csv'
    
    data_BICLARE = pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF']
    
    n_colors = len(algorithm_names)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Filter data for the specified n_agents
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define configurations to plot
    configs = [
        ('20', '30'),
        ('20', '$\\infty$'),
        ('$\\infty$', '30'),
        ('$\\infty$', '$\\infty$'),
    
    ]
    
    
    # Calculate bar width and positions
    n_configs = len(configs)
    n_algorithms = len(algorithm_names)
    group_width = 0.6  # Width for each mfr group
    bar_width = group_width / n_algorithms
    
    # Create DataFrame to store the results
    df = pd.DataFrame(columns=['mfr', 'mrl'] + algorithm_names + [f'{name} std dev' for name in algorithm_names])
    
    # Prepare data for plotting
    positions = []
    for i, (mfr, mrl) in enumerate(configs):
        config_pos = i * 1.5  # Space between configurations
        positions.append(config_pos)
        
        # Add to DataFrame
        row_data = {'mfr': mfr, 'mrl': mrl}
        
        for j, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
            # Skip BICLARE-WF for configs with mrl 30
            if algo_name == 'BICLARE-WF' and mrl == '30':
                continue
            # Filter data for the specified n_agents and noise 0
            filtered_data = data[(data['n_agents'] == f'{n_agents}_agents') & (data['noise'] == 0.0)]
            
            if not filtered_data.empty:
                # Get the column name for this configuration
                col_name = f'mfr {mfr} mrl {mrl}'
                std_col_name = f'std dev mfr {mfr} mrl {mrl}'
                
                # Get the values
                # The column names in the CSV have a different format for infinity symbol
                csv_col_name = col_name.replace('$\\infty$', '$\infty$')
                csv_std_col_name = std_col_name.replace('$\\infty$', '$\infty$')
                
                # Remove columns with 'min' 'max' 'median'
                columns = filtered_data.columns
                columns = [col for col in columns if 'min' not in col and 'max' not in col and 'median' not in col]
                
                cp_m = filtered_data[csv_col_name].values[0]
                std_dev = filtered_data[csv_std_col_name].values[0]
                
                # Add to DataFrame
                row_data[algo_name] = cp_m
                row_data[f'{algo_name} std dev'] = std_dev
                
                # Plot the bar
                ax.bar(
                    config_pos + j * bar_width - group_width/2 + bar_width/2,
                    cp_m,
                    width=bar_width,
                    color=colors[j],
                    yerr=std_dev,
                    capsize=5,
                    label=algo_name if algo_name not in ax.get_legend_handles_labels()[1] else None
                )
        
        # Add the row to DataFrame
        df = df._append(row_data, ignore_index=True)
    
    # Add vertical separators between groups
    for i in range(len(configs) - 1):
        ax.axvline(positions[i] + 0.75, color='gray', linestyle='--', alpha=0.5)
    
    # Set x-axis labels
    ax.set_xticks(positions)
    ax.set_xticklabels([f'$N_f$={mfr}, $N_s$={mrl}' for mfr, mrl in configs], fontsize=22)
    ax.set_ylim(40,90)
    
    # Set y-axis limits and labels
    ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=22)
    ax.set_ylabel('$ACP$', fontsize=25)
    # ax.set_title(f'Coverage by MFR and MRL Configuration for {n_agents} Agents', fontsize=18)
    
    # Add legend
    ax.legend(fontsize=22, ncol=2, loc='lower center')
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')
    
    # Save the plot
    # output_path = f'results_comp/accuracy_plots/{batch}/barplots/ACP_noise_0_by_mfr_mrl_{n_agents}_agents.png'
    # plt.savefig(output_path)
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/ACP_noise_0_by_mfr_mrl_{n_agents}_agents.pdf'
    plt.savefig(output_path, format="pdf", bbox_inches="tight")
    
    # Export the dataframe
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/ACP_noise_0_by_mfr_mrl_{n_agents}_agents.csv'
    df.to_csv(output_path, index=False)
    
    # plt.show()

def combine_mfr_mrl_metrics_to_csv(n_agents_list=None):
    """
    Combine MFR/MRL CP_m and F1 values for BICLARE-PP and BICLARE-WF into a single CSV file for each n_agents.
    
    Parameters:
    n_agents_list (list): List of number of agents to process. If None, uses [2,4,6,10,15].
    """
    if n_agents_list is None:
        n_agents_list = [2, 4, 6, 10, 15]
    
    # Define configurations
    configs = [
        ('$\\infty$', '$\\infty$'),
        ('$\\infty$', '30'),
        ('20', '$\\infty$'),
        ('20', '30')
    ]
    
    for n_agents in n_agents_list:
        print(f"Combining metrics for {n_agents} agents...")
        
        # Load F1 score data
        f1_file_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_mfr_mrl_{n_agents}_agents.csv'
        f1_data = pd.read_csv(f1_file_path) if os.path.exists(f1_file_path) else None
        
        # Load CP_m data
        cpm_file_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_mfr_mrl_{n_agents}_agents.csv'
        cpm_data = pd.read_csv(cpm_file_path) if os.path.exists(cpm_file_path) else None
        
        if f1_data is None or cpm_data is None:
            print(f"Missing data for {n_agents} agents, skipping...")
            continue
            
        # Create combined DataFrame
        combined_df = pd.DataFrame()
        combined_df['mfr'] = [mfr for mfr, _ in configs]
        combined_df['mrl'] = [mrl for _, mrl in configs]
        
        # Add F1 scores
        for algo in ['BICLARE-PP', 'BICLARE-WF']:
            if algo in f1_data.columns:
                combined_df[f'{algo} F1 Score'] = f1_data[algo]
                combined_df[f'{algo} F1 Std Dev'] = f1_data[f'{algo} std dev']
        
        # Add CP_m values
        for algo in ['BICLARE-PP', 'BICLARE-WF']:
            if algo in cpm_data.columns:
                combined_df[f'{algo} CP_m'] = cpm_data[algo]
                combined_df[f'{algo} CP_m Std Dev'] = cpm_data[f'{algo} std dev']
        
        # Create output directory if it doesn't exist
        output_dir = f'results_comp/accuracy_plots/{batch}/combined'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Save combined data
        output_path = f'{output_dir}/mfr_mrl_combined_metrics_{n_agents}_agents.csv'
        combined_df.to_csv(output_path, index=False)
        print(f"Combined data saved to {output_path}")

def combine_fsr_metrics_to_csv(n_agents_list=None):
    """
    Combine FSR CP_m and F1 values for BICLARE-PP and BICLARE-WF into a single CSV file for each n_agents.
    
    Parameters:
    n_agents_list (list): List of number of agents to process. If None, uses [2,4,6,10,15].
    """
    if n_agents_list is None:
        n_agents_list = [2, 4, 6, 10, 15]
    
    for n_agents in n_agents_list:
        print(f"Combining FSR metrics for {n_agents} agents...")
        
        # Load F1 score data
        f1_file_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_fsr_{n_agents}_agents.csv'
        f1_data = pd.read_csv(f1_file_path) if os.path.exists(f1_file_path) else None
        
        # Load CP_m data
        cpm_file_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_fsr_{n_agents}_agents.csv'
        cpm_data = pd.read_csv(cpm_file_path) if os.path.exists(cpm_file_path) else None
        
        if f1_data is None or cpm_data is None:
            print(f"Missing data for {n_agents} agents, skipping...")
            continue
            
        # Create combined DataFrame
        combined_df = pd.DataFrame()
        combined_df['fsr'] = f1_data['fsr']  # FSR values should be the same in both files
        
        # Add F1 scores
        for algo in ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']:
            if algo in f1_data.columns:
                combined_df[f'{algo} F1 Score'] = f1_data[algo]
                combined_df[f'{algo} F1 Std Dev'] = f1_data[f'{algo} std dev']
        
        # Add CP_m values
        for algo in ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']:
            if algo in cpm_data.columns:
                combined_df[f'{algo} CP_m'] = cpm_data[algo]
                combined_df[f'{algo} CP_m Std Dev'] = cpm_data[f'{algo} std dev']
        
        # Create output directory if it doesn't exist
        output_dir = f'results_comp/accuracy_plots/{batch}/combined'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Save combined data
        output_path = f'{output_dir}/fsr_combined_metrics_{n_agents}_agents.csv'
        combined_df.to_csv(output_path, index=False)
        print(f"Combined FSR data saved to {output_path}")

def fsr_plots_f1_comp_algorithms(n_agents):
    # Load the CSV files for different algorithms
    file_path_BICLARE = f'results_CLARE/accuracy_plots/{batch}/fsr_averaged_metrics_f1_per_fsr_noise_0__spawn_time_0.csv'
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/fsr_averaged_metrics_f1_per_fsr_noise_0__spawn_time_0.csv'
    file_path_Base_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/fsr_QT/fsr_averaged_metrics_f1_per_fsr_noise_0_spawn_time_0.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/fsr_cohesionalignment/fsr_averaged_metrics_f1_per_fsr_noise_0_spawn_time_0.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/fsr/fsr_averaged_metrics_f1_per_fsr_noise_0_spawn_time_0.csv'
    
    data_BICLARE = pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_Base_QT = pd.read_csv(file_path_Base_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_Base_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']

    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE-PP', 'BICLARE-WF']
    
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Filter data for the specified n_agents
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Create a list to store the FSR values and their corresponding metrics
    fsr_values = ['5', '15', '$\\infty$']
    
    # Calculate bar width and positions
    bar_width = 0.4/len(algorithm_datas)
    x_indexes = np.arange(len(fsr_values))

    df = pd.DataFrame()
    #add fsr column
    df['fsr'] = fsr_values
    
    for i, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
        # Filter data for the specified n_agents
        filtered_data = data[data['n_agents'] == f'{n_agents}_agents']
        
        # Extract F1 scores for each FSR value
        f1_scores = [
            filtered_data[f'fsr 5 f1_score'].values[0],
            filtered_data[f'fsr 15 f1_score'].values[0],
            filtered_data[f'fsr $\\infty$ f1_score'].values[0]
        ]
        
        # Extract standard deviations for each FSR value
        std_devs = [
            filtered_data[f'std dev fsr 5 f1_score'].values[0],
            filtered_data[f'std dev fsr 15 f1_score'].values[0],
            filtered_data[f'std dev fsr $\\infty$ f1_score'].values[0]
        ]

        #add to dataframe
        df[algo_name] = f1_scores
        df[f'{algo_name} std dev'] = std_devs
        
        # Plot bars for this algorithm
        ax.bar(
            x_indexes + i * bar_width,
            f1_scores,
            width=bar_width,
            label=algo_name,
            yerr=std_devs,
            capsize=5,
            color=colors[i]
        )
    
    # Customize the plot
    ax.set_xticks(x_indexes + bar_width * (len(algorithm_datas) - 1) / 2)
    ax.set_xticklabels(fsr_values, fontsize=16)
    ax.set_ylim(90, 100)
    # ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
    ax.set_yticklabels(ax.get_yticks(), fontsize=16)
    ax.set_xlabel('$R_f$', fontsize=18)
    ax.set_ylabel('F1 Score', fontsize=18)
    ax.set_title(f'F1 Score by FSR Value for {n_agents} Agents', fontsize=18)
    ax.legend(fontsize=14)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')
    
    # Save the plot
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_fsr_{n_agents}_agents.png'
    plt.savefig(output_path)
    
    #export the dataframe
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/f1_score_by_fsr_{n_agents}_agents.csv'
    df.to_csv(output_path, index=False)

    # plt.show()

def fsr_plots_f1_comp_algorithms_per_map(n_agents):
    for env_map in ['house_tilted', 'office_tilted', 'museum_tilted']:
        # Load the CSV files for different algorithms
        file_path_BICLARE = f'results_CLARE/accuracy_plots/{batch}/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_Base_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/fsr_QT/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/fsr_cohesionalignment/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/fsr/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'

        data_BICLARE = pd.read_csv(file_path_BICLARE)
        data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
        data_Base_QT = pd.read_csv(file_path_Base_QT)
        data_BASE = pd.read_csv(file_path_BASE)
        data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)

        algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_Base_QT, data_BASE, data_BASE_NCA]
        algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']

        # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
        # algorithm_names = ['BICLARE-PP', 'BICLARE-WF']

        n_colors = len(algorithm_datas)
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in range(n_colors)]
        colors = [lighten_color(c, lighten_factor) for c in colors]    

        # Filter data for the specified n_agents
        fig, ax = plt.subplots(figsize=(10, 6))

        # Create a list to store the FSR values and their corresponding metrics
        fsr_values = ['5', '15', '$\\infty$']

        # Calculate bar width and positions
        bar_width = 0.4/len(algorithm_datas)
        x_indexes = np.arange(len(fsr_values))

        df = pd.DataFrame()
        #add fsr column
        df['fsr'] = fsr_values

        for i, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
            # Filter data for the specified n_agents
            filtered_data = data[data['n_agents'] == f'{n_agents}_agents']

            # Extract F1 scores for each FSR value
            recalls = [
                filtered_data[f'fsr 5 recall'].values[0],
                filtered_data[f'fsr 15 recall'].values[0],
                filtered_data[f'fsr $\\infty$ recall'].values[0]
            ]

            # Extract F1 scores for each FSR value
            precisions = [
                filtered_data[f'fsr 5 precision'].values[0],
                filtered_data[f'fsr 15 precision'].values[0],
                filtered_data[f'fsr $\\infty$ precision'].values[0]
            ]

            #calculate f1 score
            f1_scores = []
            for recall, precision in zip(recalls, precisions):
                if recall + precision == 0:
                    f1_scores.append(0)
                else:
                    f1 = (2 * recall * precision) / (recall + precision)
                    f1_scores.append(f1)

            #calculate std devs
            recalls_std = [
                filtered_data[f'fsr 5 recall std dev'].values[0],
                filtered_data[f'fsr 15 recall std dev'].values[0],
                filtered_data[f'fsr $\\infty$ recall std dev'].values[0]
            ]

            precisions_std = [
                filtered_data[f'fsr 5 precision std dev'].values[0],
                filtered_data[f'fsr 15 precision std dev'].values[0],
                filtered_data[f'fsr $\\infty$ precision std dev'].values[0]
            ]

            std_devs_f1 = []
            for r, p, r_std, p_std in zip(recalls, precisions, recalls_std, precisions_std):
                if r + p == 0:
                    std_devs_f1.append(0)
                else:
                    df_dr = (2 * p**2) / (r + p)**2
                    df_dp = (2 * r**2) / (r + p)**2
                    variance_f1 = (df_dr**2) * (r_std**2) + (df_dp**2) * (p_std**2)
                    std_dev_f1 = np.sqrt(variance_f1)
                    std_devs_f1.append(std_dev_f1)
            
            # # Extract F1 scores for each FSR value
            # f1_scores = [
            #     filtered_data[f'fsr 5 f1_score'].values[0],
            #     filtered_data[f'fsr 15 f1_score'].values[0],
            #     filtered_data[f'fsr $\\infty$ f1_score'].values[0]
            # ]
            
            # Extract standard deviations for each FSR value
            # std_devs = [
            #     filtered_data[f'std dev fsr 5 f1_score'].values[0],
            #     filtered_data[f'std dev fsr 15 f1_score'].values[0],
            #     filtered_data[f'std dev fsr $\\infty$ f1_score'].values[0]
            # ]

            #add to dataframe
            df[algo_name] = f1_scores
            df[f'{algo_name} std dev'] = std_devs_f1
            
            # Plot bars for this algorithm
            ax.bar(
                x_indexes + i * bar_width,
                f1_scores,
                width=bar_width,
                label=algo_name,
                yerr=std_devs_f1,
                capsize=5,
                color=colors[i]
            )

        # Customize the plot
        ax.set_xticks(x_indexes + bar_width * (len(algorithm_datas) - 1) / 2)
        ax.set_xticklabels(fsr_values, fontsize=16)
        ax.set_ylim(90, 100)
        # ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
        ax.set_yticklabels(ax.get_yticks(), fontsize=16)
        ax.set_xlabel('$R_f$', fontsize=18)
        ax.set_ylabel('F1 Score', fontsize=18)
        ax.set_title(f'F1 Score by FSR Value for {n_agents} Agents', fontsize=18)
        ax.legend(fontsize=14)
        plt.grid(axis='y', linestyle='--', alpha=0.7)

        plt.tight_layout()

        # Create directory if it doesn't exist
        if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
            os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')

        # Save the plot
        output_path = f'results_comp/accuracy_plots/{batch}/barplots/{env_map}_f1_score_by_fsr_{n_agents}_agents.png'
        plt.savefig(output_path)

        #export the dataframe
        output_path = f'results_comp/accuracy_plots/{batch}/barplots/{env_map}_f1_score_by_fsr_{n_agents}_agents.csv'
        df.to_csv(output_path, index=False)

        # plt.show()


def fsr_plots_cpm_noise_0_comp_algorithms(n_agents):
    # Load the CSV files for different algorithms
    file_path_BICLARE = f'results_CLARE/accuracy_plots/{batch}/fsr_averaged_metrics_cp_m_per_agents_noise_0_per_fsr_spawn_time_0.csv'
    file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/fsr_averaged_metrics_cp_m_per_agents_noise_0_per_fsr_spawn_time_0.csv'
    file_path_Base_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/fsr_QT/fsr_averaged_metrics_cp_m_per_agents_noise_0_per_fsr_spawn_time_0.csv'
    file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/fsr_cohesionalignment/fsr_averaged_metrics_cp_m_per_agents_noise_0_per_fsr_spawn_time_0.csv'
    file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/fsr/fsr_averaged_metrics_cp_m_per_agents_noise_0_per_fsr_spawn_time_0.csv'
    
    data_BICLARE = pd.read_csv(file_path_BICLARE)
    data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
    data_Base_QT = pd.read_csv(file_path_Base_QT)
    data_BASE = pd.read_csv(file_path_BASE)
    data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)
    
    algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_Base_QT, data_BASE, data_BASE_NCA]
    algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']

    # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
    # algorithm_names = ['BICLARE-PP', 'BICLARE-WF']
    
    n_colors = len(algorithm_datas)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(n_colors)]
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Filter data for the specified n_agents
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Create a list to store the FSR values and their corresponding metrics
    fsr_values = ['5', '15', '$\\infty$']
    
    # Calculate bar width and positions
    bar_width = 0.4/len(algorithm_datas)
    x_indexes = np.arange(len(fsr_values))

    df = pd.DataFrame()
    #add fsr column
    df['fsr'] = fsr_values
    
    for i, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
        # Filter data for the specified n_agents
        filtered_data = data[data['n_agents'] == f'{n_agents}_agents']
        
        # Extract coverage values for each FSR value
        coverage_values = [
            filtered_data[f'fsr 5 coverage'].values[0],
            filtered_data[f'fsr 15 coverage'].values[0],
            filtered_data[f'fsr $\\infty$ coverage'].values[0]
        ]
        
        # Extract standard deviations for each FSR value
        std_devs = [
            filtered_data[f'std dev fsr 5 coverage'].values[0],
            filtered_data[f'std dev fsr 15 coverage'].values[0],
            filtered_data[f'std dev fsr $\\infty$ coverage'].values[0]
        ]

        #add to dataframe
        df[algo_name] = coverage_values
        df[f'{algo_name} std dev'] = std_devs
        
        # Plot bars for this algorithm
        ax.bar(
            x_indexes + i * bar_width,
            coverage_values,
            width=bar_width,
            label=algo_name,
            yerr=std_devs,
            capsize=5,
            color=colors[i]
        )
    
    # Customize the plot
    ax.set_xticks(x_indexes + bar_width * (len(algorithm_datas) - 1) / 2)
    ax.set_xticklabels(fsr_values, fontsize=16)
    # ax.set_ylim(50, 110)
    ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
    ax.set_xlabel('$R_f$', fontsize=18)
    ax.set_ylabel('$CP_m$', fontsize=18)
    # ax.set_title(f'Coverage by FSR Value for {n_agents} Agents', fontsize=18)
    ax.legend(fontsize=18)
    plt.grid(axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()
    
    # Create directory if it doesn't exist
    if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
        os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')
    
    # Save the plot
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_fsr_{n_agents}_agents.png'
    plt.savefig(output_path)

    #export the dataframe
    output_path = f'results_comp/accuracy_plots/{batch}/barplots/coverage_noise_0_by_fsr_{n_agents}_agents.csv'
    df.to_csv(output_path, index=False)
    # plt.show()

def fsr_plots_cpm_comp_algorithms_per_map(n_agents):
    for env_map in ['house_tilted', 'office_tilted', 'museum_tilted']:
        # Load the CSV files for different algorithms
        file_path_BICLARE = f'results_CLARE/accuracy_plots/{batch}/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_BICLARE_wallfollowing = f'results_CLARE_wallfollowing/accuracy_plots/fsr_mfr/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_Base_QT = f'../Thesis_ARGoS2/results_Base_QT/accuracy_plots/fsr_QT/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_BASE = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/fsr_cohesionalignment/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'
        file_path_BASE_NCA = f'../Thesis_ARGoS2/results_Base/accuracy_plots/fsr/csv_tables/fsr_accuracy_{env_map}_noise_0.0_spawn_time_0.csv'

        data_BICLARE = pd.read_csv(file_path_BICLARE)
        data_BICLARE_wallfollowing = pd.read_csv(file_path_BICLARE_wallfollowing)
        data_Base_QT = pd.read_csv(file_path_Base_QT)
        data_BASE = pd.read_csv(file_path_BASE)
        data_BASE_NCA = pd.read_csv(file_path_BASE_NCA)

        algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing, data_Base_QT, data_BASE, data_BASE_NCA]
        algorithm_names = ['BICLARE-PP', 'BICLARE-WF', 'FSP-QT', 'FSP-G', 'FSP']

        # algorithm_datas = [data_BICLARE, data_BICLARE_wallfollowing]
        # algorithm_names = ['BICLARE-PP', 'BICLARE-WF']

        n_colors = len(algorithm_datas)
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in range(n_colors)]
        colors = [lighten_color(c, lighten_factor) for c in colors]    

        # Filter data for the specified n_agents
        fig, ax = plt.subplots(figsize=(10, 6))

        # Create a list to store the FSR values and their corresponding metrics
        fsr_values = ['5', '15', '$\\infty$']

        # Calculate bar width and positions
        bar_width = 0.4/len(algorithm_datas)
        x_indexes = np.arange(len(fsr_values))

        df = pd.DataFrame()
        #add fsr column
        df['fsr'] = fsr_values

        for i, (data, algo_name) in enumerate(zip(algorithm_datas, algorithm_names)):
            # Filter data for the specified n_agents
            filtered_data = data[data['n_agents'] == f'{n_agents}_agents']

            # Extract F1 scores for each FSR value
            coverages = [
                filtered_data[f'fsr 5 coverage'].values[0],
                filtered_data[f'fsr 15 coverage'].values[0],
                filtered_data[f'fsr $\\infty$ coverage'].values[0]
            ]

        

            #calculate std devs
            coverages_std = [
                filtered_data[f'fsr 5 recall std dev'].values[0],
                filtered_data[f'fsr 15 recall std dev'].values[0],
                filtered_data[f'fsr $\\infty$ recall std dev'].values[0]
            ]

        
            
            
            #add to dataframe
            df[algo_name] = coverages
            df[f'{algo_name} std dev'] = coverages_std
            
            # Plot bars for this algorithm
            ax.bar(
                x_indexes + i * bar_width,
                coverages,
                width=bar_width,
                label=algo_name,
                yerr=coverages_std,
                capsize=5,
                color=colors[i]
            )

        # Customize the plot
        ax.set_xticks(x_indexes + bar_width * (len(algorithm_datas) - 1) / 2)
        ax.set_xticklabels(fsr_values, fontsize=16)
        # ax.set_ylim(90, 100)
        # ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
        ax.set_yticklabels(ax.get_yticks(), fontsize=16)
        ax.set_xlabel('$R_f$', fontsize=18)
        ax.set_ylabel('$CP_m$ (%)', fontsize=18)
        # ax.set_title(f'F1 Score by FSR Value for {n_agents} Agents', fontsize=18)
        ax.legend(fontsize=18)
        plt.grid(axis='y', linestyle='--', alpha=0.7)

        plt.tight_layout()

        # Create directory if it doesn't exist
        if not os.path.exists(f'results_comp/accuracy_plots/{batch}/barplots'):
            os.makedirs(f'results_comp/accuracy_plots/{batch}/barplots')

        # Save the plot
        output_path = f'results_comp/accuracy_plots/{batch}/barplots/{env_map}_cpm_score_by_fsr_{n_agents}_agents.png'
        plt.savefig(output_path)

        #export the dataframe
        output_path = f'results_comp/accuracy_plots/{batch}/barplots/{env_map}_cpm_score_by_fsr_{n_agents}_agents.csv'
        df.to_csv(output_path, index=False)

        # plt.show()

def combine_cpm_scores_by_fsr(n_agents_list=None, env_maps=None):
    """
    Combine all {env_map}_cpm_score_by_fsr_{n_agents}_agents.csv files into one CSV file.
    
    Parameters:
    n_agents_list (list): List of number of agents to process. If None, uses [2,4,6,10,15].
    env_maps (list): List of environment maps to process. If None, uses ['house_tilted', 'office_tilted', 'museum_tilted'].
    """
    if n_agents_list is None:
        n_agents_list = [2, 4, 6, 10, 15]
    if env_maps is None:
        env_maps = ['house_tilted', 'office_tilted', 'museum_tilted']
    
    combined_df = pd.DataFrame()
    
    for n_agents in n_agents_list:
        for env_map in env_maps:
            file_path = f'results_comp/accuracy_plots/{batch}/barplots/{env_map}_cpm_score_by_fsr_{n_agents}_agents.csv'
            if os.path.exists(file_path):
                df = pd.read_csv(file_path)
                df['n_agents'] = n_agents
                df['env_map'] = env_map
                combined_df = pd.concat([combined_df, df], ignore_index=True)
            else:
                print(f"File not found: {file_path}")
    
    # Create output directory if it doesn't exist
    output_dir = f'results_comp/accuracy_plots/{batch}/combined'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save combined data
    output_path = f'{output_dir}/combined_cpm_scores_by_fsr.csv'
    combined_df.to_csv(output_path, index=False)
    print(f"Combined data saved to {output_path}")

def combine_f1_scores_by_fsr(n_agents_list=None, env_maps=None):
    """
    Combine all {env_map}_f1_score_by_fsr_{n_agents}_agents.csv files into one CSV file.
    
    Parameters:
    n_agents_list (list): List of number of agents to process. If None, uses [2,4,6,10,15].
    env_maps (list): List of environment maps to process. If None, uses ['house_tilted', 'office_tilted', 'museum_tilted'].
    """
    if n_agents_list is None:
        n_agents_list = [2, 4, 6, 10, 15]
    if env_maps is None:
        env_maps = ['house_tilted', 'office_tilted', 'museum_tilted']
    
    combined_df = pd.DataFrame()
    
    for n_agents in n_agents_list:
        for env_map in env_maps:
            file_path = f'results_comp/accuracy_plots/{batch}/barplots/{env_map}_f1_score_by_fsr_{n_agents}_agents.csv'
            if os.path.exists(file_path):
                df = pd.read_csv(file_path)
                df['n_agents'] = n_agents
                df['env_map'] = env_map
                combined_df = pd.concat([combined_df, df], ignore_index=True)
            else:
                print(f"File not found: {file_path}")
    
    # Create output directory if it doesn't exist
    output_dir = f'results_comp/accuracy_plots/{batch}/combined'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save combined data
    output_path = f'{output_dir}/combined_f1_scores_by_fsr.csv'
    combined_df.to_csv(output_path, index=False)
    print(f"Combined data saved to {output_path}")



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

lighten_factor = 0.2

# barplots_per_n_agents_per_map()
# barplots_overall()
# barplots_f1_overall_comp_algorithms()
# barplots_cpm_overall_comp_algorithms()
# mfr_mrl_plots()

for n_agents in [2]:#,4,6,10,15]:

#     # fsr_plots_f1_comp_algorithms(n_agents)
    mfr_mrl_plots_cpm_noise_0_comp_algorithms(n_agents)
    mfr_mrl_plots_f1_comp_algorithms(n_agents)
    # mfr_mrl_plots_f1_comp_algorithms(n_agents)
    # mfr_mrl_plots_cpm_noise_0_comp_algorithms(n_agents)
    # mfr_mrl_plots_ACP_noise_0_comp_algorithms(n_agents)

mfr_mrl_plots_ACP_noise_0_comp_algorithms(6)

#     fsr_plots_f1_comp_algorithms_per_map(n_agents)
#     fsr_plots_cpm_comp_algorithms_per_map(n_agents)
#     plt.close()

# combine_mfr_mrl_metrics_to_csv()
# combine_fsr_metrics_to_csv()

# agent_list = [2,6,15]

# combine_cpm_scores_by_fsr(agent_list)
# combine_f1_scores_by_fsr(agent_list)