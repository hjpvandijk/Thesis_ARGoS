import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

def plot_spawn_time_evap_comparison(metric='f1', n_agents=10):
    """
    Creates a plot showing the effect of spawn_time and evaporation on the specified metric.
    
    Parameters:
    metric (str): 'f1', 'cpm', or 'acp'
    n_agents (int): Number of agents to display data for
    """
    # Set up figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define spawn times to plot and their labels
    spawn_times = [100, 180, 0]
    spawn_time_labels = ['100s', '180s', '$\\infty$']
    
    # Define evaporation values that might be in the data
    evap_values = ['50', '100', '150', '$\\infty$']
    
    # Set up colors and markers
    colors = plt.cm.tab10(np.linspace(0, 1, len(evap_values)))
    
    # Keep track of lines for legend
    legend_elements = []
    
    # For each evaporation value, plot a line across spawn times
    for i, evap in enumerate(evap_values):
        data_points = []
        std_devs = []
        
        for spawn_time in spawn_times:
            # Load data file based on metric type
            if metric == 'f1':
                file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
                column_name = f'evap {evap} f1_score'
                std_dev_column = f'std dev evap {evap} f1_score'
                y_label = 'F1 Score (%)'
                title = f'F1 Score vs. Spawn Time and Evaporation ({n_agents} agents)'
            elif metric == 'cpm':
                file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
                column_name = f'evap {evap} coverage'
                std_dev_column = f'std dev evap {evap} coverage'
                y_label = '$CP_m$ (%)'
                title = f'Coverage ($CP_m$) vs. Spawn Time and Evaporation ({n_agents} agents)'
            elif metric == 'acp':
                file_path = f'results_CLARE/coverage_plots/dynam_and_evap/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
                column_name = f'evap {evap} coverage'
                std_dev_column = f'std dev evap {evap} coverage'
                y_label = '$ACP$ (%)'
                title = f'Area Coverage ($ACP$) vs. Spawn Time and Evaporation ({n_agents} agents)'
            
            if os.path.exists(file_path):
                df = pd.read_csv(file_path)
                agent_row = df[df['n_agents'] == f'{n_agents}_agents']
                
                if not agent_row.empty and column_name in agent_row.columns:
                    data_points.append(agent_row[column_name].values[0])
                    if std_dev_column in agent_row.columns:
                        std_devs.append(agent_row[std_dev_column].values[0])
                    else:
                        std_devs.append(0)
                else:
                    # Handle missing data
                    data_points.append(np.nan)
                    std_devs.append(0)
            else:
                # Handle missing file
                data_points.append(np.nan)
                std_devs.append(0)
        
        # Plot the line for this evaporation value if we have data
        if not all(np.isnan(data_points)):
            line = ax.errorbar(
                spawn_times, 
                data_points, 
                yerr=std_devs, 
                color=colors[i], 
                marker='o', 
                linestyle='-', 
                linewidth=2,
                markersize=8,
                capsize=5,
                label=f'Evap. Time = {evap}',
                alpha=0.8
            )
            legend_elements.append(line)
    
    # Compare with FSP-G if available (only for metric='f1')
    if metric == 'f1':
        try:
            fsp_data = []
            fsp_std_devs = []
            
            for spawn_time in spawn_times:
                # Try to load FSP-G data - this is just a placeholder, adapt to your actual data structure
                fsp_file_path = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/dynam_and_evap/fsp_metrics_spawn_time_{spawn_time}.csv'
                if os.path.exists(fsp_file_path):
                    fsp_df = pd.read_csv(fsp_file_path)
                    fsp_agent_row = fsp_df[fsp_df['n_agents'] == f'{n_agents}_agents']
                    
                    if not fsp_agent_row.empty:
                        fsp_data.append(fsp_agent_row['f1_score'].values[0])
                        fsp_std_devs.append(fsp_agent_row['std dev f1_score'].values[0])
                    else:
                        fsp_data.append(np.nan)
                        fsp_std_devs.append(0)
                else:
                    fsp_data.append(np.nan)
                    fsp_std_devs.append(0)
            
            if not all(np.isnan(fsp_data)):
                fsp_line = ax.errorbar(
                    spawn_times, 
                    fsp_data, 
                    yerr=fsp_std_devs, 
                    color='black', 
                    marker='s', 
                    linestyle='--', 
                    linewidth=2,
                    markersize=8,
                    capsize=5,
                    label='FSP-G',
                    alpha=0.8
                )
                legend_elements.append(fsp_line)
        except Exception as e:
            print(f"Could not load FSP-G data: {e}")
    
    # Set x-axis labels and ticks
    ax.set_xticks(spawn_times)
    ax.set_xticklabels(spawn_time_labels, fontsize=16)
    
    # Set y-axis limits based on metric
    if metric == 'f1':
        ax.set_ylim(96, 99)
    elif metric == 'cpm':
        ax.set_ylim(25, 106)
    elif metric == 'acp':
        ax.set_ylim(25, 106)
    
    # Set axis labels and title
    ax.set_xlabel('Spawn Time', fontsize=18)
    ax.set_ylabel(y_label, fontsize=18)
    ax.set_title(title, fontsize=18)
    
    # Add grid and legend
    # ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=18, loc='best')
    
    # Add y-tick labels
    ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=14)
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    output_dir = f'results_CLARE/accuracy_plots/dynam_and_evap/plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save the plot
    plt.savefig(f'{output_dir}/{metric}_spawn_time_evap_{n_agents}_agents.png', dpi=300)
    plt.close()
    
    print(f"Plot saved to {output_dir}/{metric}_spawn_time_evap_{n_agents}_agents.png")

def plot_evap_comparison_per_spawn_time(metric='f1', spawn_time=0):
    """
    Creates a bar chart showing the effect of evaporation for a specific spawn_time.
    
    Parameters:
    metric (str): 'f1', 'cpm', or 'acp'
    spawn_time (int): Spawn time to display data for (0, 100, or 180)
    """
    # Set up figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define evaporation values and their labels
    evap_values = ['50', '100', '150', '$\\infty$']
    
    # Load data file based on metric type
    if metric == 'f1':
        file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
        metric_name = 'f1_score'
        y_label = 'F1 Score (%)'
        title = f'F1 Score vs. Evaporation Time (Spawn Time = {spawn_time}s)'
    elif metric == 'cpm':
        file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        metric_name = 'coverage'
        y_label = '$CP_m$ (%)'
        title = f'Coverage ($CP_m$) vs. Evaporation Time (Spawn Time = {spawn_time}s)'
    elif metric == 'acp':
        file_path = f'results_CLARE/coverage_plots/dynam_and_evap/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        metric_name = 'coverage'
        y_label = '$ACP$ (%)'
        title = f'Area Coverage ($ACP$) vs. Evaporation Time (Spawn Time = {spawn_time}s)'
    
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return
    
    df = pd.read_csv(file_path)
    
    # Define agent counts to show
    agent_counts = [2, 4, 6, 10, 15]
    
    # Set up bar positions and width
    n_agents = len(agent_counts)
    n_evap = len(evap_values)
    bar_width = 0.8 / n_evap
    
    # Set colors
    colors = plt.cm.tab10(np.linspace(0, 1, n_evap))
    
    # Plot bars for each agent count and evaporation time
    for i, evap in enumerate(evap_values):
        values = []
        errors = []
        
        for agent_count in agent_counts:
            agent_row = df[df['n_agents'] == f'{agent_count}_agents']
            
            if not agent_row.empty:
                col_name = f'evap {evap} {metric_name}'
                std_col_name = f'std dev evap {evap} {metric_name}'
                
                if col_name in agent_row.columns:
                    values.append(agent_row[col_name].values[0])
                    
                    if std_col_name in agent_row.columns:
                        errors.append(agent_row[std_col_name].values[0])
                    else:
                        errors.append(0)
                else:
                    values.append(0)
                    errors.append(0)
            else:
                values.append(0)
                errors.append(0)
        
        # Calculate bar positions
        x = np.arange(n_agents)
        positions = x - 0.4 + (i + 0.5) * bar_width
        
        # Plot bars
        ax.bar(
            positions, 
            values, 
            width=bar_width, 
            yerr=errors, 
            capsize=5, 
            label=f'Evap Time = {evap}',
            color=colors[i],
            alpha=0.8
        )
    
    # Set x-axis labels and ticks
    ax.set_xticks(np.arange(n_agents))
    ax.set_xticklabels([f'{count} Agents' for count in agent_counts], fontsize=14)
    
    # Set y-axis limits based on metric
    if metric == 'f1':
        ax.set_ylim(96, 99)
    elif metric == 'cpm':
        ax.set_ylim(25, 106)
    elif metric == 'acp':
        ax.set_ylim(25, 106)
    
    # Set axis labels and title
    ax.set_xlabel('Number of Agents', fontsize=16)
    ax.set_ylabel(y_label, fontsize=16)
    # ax.set_title(title, fontsize=18)
    
    # Add grid and legend
    # ax.grid(True, linestyle='--', alpha=0.6, axis='y')
    ax.legend(fontsize=18, loc='best')
    
    # Add y-tick labels
    ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=14)
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    output_dir = f'results_CLARE/accuracy_plots/dynam_and_evap/plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save the plot
    plt.savefig(f'{output_dir}/{metric}_evap_comparison_spawn_time_{spawn_time}.png', dpi=300)
    plt.close()
    
    print(f"Plot saved to {output_dir}/{metric}_evap_comparison_spawn_time_{spawn_time}.png")

def plot_spawn_time_comparison(metric='f1', evap_value=None):
    """
    Creates a bar chart comparing different spawn times for a specific evaporation value.
    
    Parameters:
    metric (str): 'f1', 'cpm', or 'acp'
    evap_value (str): Evaporation value to display data for (e.g., '10', '20', '160', '$\\infty$')
                     If None, uses '$\\infty$'
    """
    # Set default evap value if not provided
    if evap_value is None:
        evap_value = '$\\infty$'
    
    # Set up figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define spawn times and their labels
    spawn_times = [100, 180, 0]
    spawn_time_labels = ['100s', '180s', '$\\infty$']
    
    # Define agent counts to show
    agent_counts = [2, 4, 6, 10, 15]
    
    # Set up colors
    colors = plt.cm.viridis(np.linspace(0, 1, len(spawn_times)))
    
    # Set up bar positions and width
    n_agents = len(agent_counts)
    n_spawn_times = len(spawn_times)
    bar_width = 0.8 / n_spawn_times
    
    # Plot bars for each agent count and spawn time
    for i, spawn_time in enumerate(spawn_times):
        # Load data file based on metric type
        if metric == 'f1':
            file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
            metric_name = 'f1_score'
            y_label = 'F1 Score (%)'
            title = f'F1 Score vs. Spawn Time (Evap. Time = {evap_value})'
        elif metric == 'cpm':
            file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
            metric_name = 'coverage'
            y_label = '$CP_m$ (%)'
            title = f'Coverage ($CP_m$) vs. Spawn Time (Evap. Time = {evap_value})'
        elif metric == 'acp':
            file_path = f'results_CLARE/coverage_plots/dynam_and_evap/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
            metric_name = 'coverage'
            y_label = '$ACP$ (%)'
            title = f'Area Coverage ($ACP$) vs. Spawn Time (Evap. Time = {evap_value})'
        
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            continue
        
        df = pd.read_csv(file_path)
        
        values = []
        errors = []
        
        for agent_count in agent_counts:
            agent_row = df[df['n_agents'] == f'{agent_count}_agents']
            
            if not agent_row.empty:
                col_name = f'evap {evap_value} {metric_name}'
                std_col_name = f'std dev evap {evap_value} {metric_name}'
                
                if col_name in agent_row.columns:
                    values.append(agent_row[col_name].values[0])
                    
                    if std_col_name in agent_row.columns:
                        errors.append(agent_row[std_col_name].values[0])
                    else:
                        errors.append(0)
                else:
                    values.append(0)
                    errors.append(0)
            else:
                values.append(0)
                errors.append(0)
        
        # Calculate bar positions
        x = np.arange(n_agents)
        positions = x - 0.4 + (i + 0.5) * bar_width
        
        # Plot bars
        ax.bar(
            positions, 
            values, 
            width=bar_width, 
            yerr=errors, 
            capsize=5, 
            label=f'Spawn Time = {spawn_time_labels[i]}',
            color=colors[i],
            alpha=0.8
        )
    
    # Compare with FSP-G if available (only for metric='f1')
    if metric == 'f1':
        try:
            # Load FSP-G data for different spawn times
            for i, spawn_time in enumerate(spawn_times):
                fsp_file_path = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/dynam_and_evap/fsp_metrics_spawn_time_{spawn_time}.csv'
                
                if os.path.exists(fsp_file_path):
                    fsp_df = pd.read_csv(fsp_file_path)
                    
                    fsp_values = []
                    fsp_errors = []
                    
                    for agent_count in agent_counts:
                        fsp_agent_row = fsp_df[fsp_df['n_agents'] == f'{agent_count}_agents']
                        
                        if not fsp_agent_row.empty and 'f1_score' in fsp_agent_row.columns:
                            fsp_values.append(fsp_agent_row['f1_score'].values[0])
                            
                            if 'std dev f1_score' in fsp_agent_row.columns:
                                fsp_errors.append(fsp_agent_row['std dev f1_score'].values[0])
                            else:
                                fsp_errors.append(0)
                        else:
                            fsp_values.append(0)
                            fsp_errors.append(0)
                    
                    # Plot FSP-G bars with hatched pattern
                    x = np.arange(n_agents)
                    positions = x - 0.4 + (i + 0.5) * bar_width
                    
                    # Add FSP-G bar as a smaller bar on top of the BICLARE-PP bar
                    ax.bar(
                        positions, 
                        fsp_values, 
                        width=bar_width/1.5, 
                        yerr=fsp_errors, 
                        capsize=5, 
                        label=f'FSP-G (Spawn={spawn_time_labels[i]})' if i == 0 else None,
                        color='none',
                        edgecolor='black',
                        linestyle='--',
                        hatch='////',
                        alpha=0.8
                    )
        except Exception as e:
            print(f"Could not load FSP-G data: {e}")
    
    # Set x-axis labels and ticks
    ax.set_xticks(np.arange(n_agents))
    ax.set_xticklabels([f'{count} Agents' for count in agent_counts], fontsize=16)
    
    # Set y-axis limits based on metric
    if metric == 'f1':
        ax.set_ylim(96, 99)
    elif metric == 'cpm':
        ax.set_ylim(25, 106)
    elif metric == 'acp':
        ax.set_ylim(25, 106)
    
    # Set axis labels and title
    ax.set_xlabel('Number of Agents', fontsize=16)
    ax.set_ylabel(y_label, fontsize=16)
    # ax.set_title(title, fontsize=18)
    
    # Add grid and legend
    # ax.grid(True, linestyle='--', alpha=0.6, axis='y')
    ax.legend(fontsize=18, loc='best')
    
    # Add y-tick labels
    ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
    
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    output_dir = f'results_CLARE/accuracy_plots/dynam_and_evap/plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save the plot with evap value in filename (replace special characters)
    safe_evap = evap_value.replace('$', '').replace('\\', '').replace('{', '').replace('}', '')
    plt.savefig(f'{output_dir}/{metric}_spawn_time_comparison_evap_{safe_evap}.png', dpi=300)
    plt.close()
    
    print(f"Plot saved to {output_dir}/{metric}_spawn_time_comparison_evap_{safe_evap}.png")

def plot_spawn_time_comparison_facet(metric='f1'):
    """
    Creates a faceted bar chart comparing all spawn times for all evaporation values.
    
    Parameters:
    metric (str): 'f1', 'cpm', or 'acp'
    """
    # Define spawn times and their labels
    spawn_times = [100, 180, 0]
    spawn_time_labels = ['100s', '180s', '$\\infty$']
    
    # Define evaporation values and their labels
    evap_values = ['50', '100', '150', '$\\infty$']
    
    # Define agent counts to show
    agent_counts = [2, 4, 6, 10, 15]
    
    # Set labels based on metric
    if metric == 'f1':
        metric_name = ' f1_score'
        y_label = 'F1 Score (%)'
        title = f'F1 Score Comparison Across Spawn Times and Evaporation Values'
    elif metric == 'cpm':
        metric_name = ' coverage'
        y_label = '$CP_m$ (%)'
        title = f'Coverage ($CP_m$) Comparison Across Spawn Times and Evaporation Values'
    elif metric == 'acp':
        metric_name = ''
        y_label = '$ACP$ (%)'
        title = f'Area Coverage ($ACP$) Comparison Across Spawn Times and Evaporation Values'
    
    # Calculate dimensions for the facet grid
    n_agents = len(agent_counts)
    n_evap = len(evap_values)
    n_spawn = len(spawn_times)
    
    # Set up a grid of subplots - one row for each agent count
    fig, axes = plt.subplots(n_agents, 1, figsize=(14, 4*n_agents), sharex=True)
    
    # Set colors for spawn times
    colors = plt.cm.viridis(np.linspace(0, 1, n_spawn))
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Bar width and spacing
    bar_width = 0.8 / n_spawn
    
    # Collect all data first
    data = {}
    for spawn_time in spawn_times:
        if metric == 'f1':
            file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
        elif metric == 'acp':
            file_path = f'results_CLARE/coverage_plots/dynam_and_evap/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        elif metric == 'cpm':
            file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        
        if os.path.exists(file_path):
            data[spawn_time] = pd.read_csv(file_path)
        else:
            print(f"File not found: {file_path}")
    
    # For each agent count (row in the facet)
    for row, agent_count in enumerate(agent_counts):
        ax = axes[row]
        
        # For each evaporation value (group on x-axis)
        for e, evap in enumerate(evap_values):
            # For each spawn time (bar within group)
            for s, spawn_time in enumerate(spawn_times):
                if spawn_time in data:
                    df = data[spawn_time]
                    agent_row = df[df['n_agents'] == f'{agent_count}_agents']
                    
                    if not agent_row.empty:
                        col_name = f'evap {evap}{metric_name}'
                        std_col_name = f'std dev evap {evap}{metric_name}'
                        
                        value = agent_row[col_name].values[0] if col_name in agent_row.columns else 0
                        error = agent_row[std_col_name].values[0] if std_col_name in agent_row.columns else 0
                        
                        # Calculate bar position
                        x_pos = e + (s - 1) * bar_width
                        
                        # Plot bar
                        ax.bar(
                            x_pos, 
                            value, 
                            width=bar_width*0.9, 
                            yerr=error, 
                            capsize=3, 
                            color=colors[s],
                            alpha=0.8
                        )
        
        # Customize row subplot
        ax.set_title(f'{agent_count} Agents', fontsize=20)
        ax.set_ylabel(y_label, fontsize=20)
        
        # Add grid
        # ax.grid(True, linestyle='--', alpha=0.6, axis='y')
        
        # Set y-axis limits based on metric
        if metric == 'f1':
            ax.set_ylim(96, 99)
        elif metric == 'cpm':
            ax.set_ylim(25, 106)
        elif metric == 'acp':
            ax.set_ylim(20, 106)
        
        # Format y-tick labels
        if metric == 'f1':
            ax.set_yticklabels([f"{y}" for y in ax.get_yticks()], fontsize=18)
        else:
            ax.set_yticklabels([f"{int(y)}" for y in ax.get_yticks()], fontsize=18)
    
    # Set the x-tick positions and labels (only for the bottom subplot)
    axes[-1].set_xticks(np.arange(len(evap_values)))
    axes[-1].set_xticklabels([f'$T_{{\\varphi}}$={e}' for e in evap_values], fontsize=20)
    axes[-1].set_xlabel('Evaporation Time', fontsize=21)
    
    # Add a common legend for spawn times
    handles = [plt.Rectangle((0,0),1,1, color=colors[s]) for s in range(n_spawn)]
    fig.legend(handles, spawn_time_labels, title='Spawn Time', loc='upper center', 
            ncol=n_spawn, fontsize=20, bbox_to_anchor=(0.5, 0.98), title_fontsize=20)
    
    # Add overall title
    # fig.suptitle(title, fontsize=18)
    
    # Adjust layout
    plt.tight_layout(rect=[0, 0.02, 1, 0.92])
    
    # Create directory if it doesn't exist
    output_dir = f'results_CLARE/accuracy_plots/dynam_and_evap/plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save the plot
    plt.savefig(f'{output_dir}/{metric}_facet_comparison_all.png', dpi=300)
    plt.close()
    
    print(f"Faceted plot saved to {output_dir}/{metric}_facet_comparison_all.png")

def plot_spawn_time_comparison_facet_comp_algs(metric='f1'):
    """
    Creates a faceted bar chart comparing all spawn times for all evaporation values.
    
    Parameters:
    metric (str): 'f1', 'cpm', or 'acp'
    """
    # Define spawn times and their labels
    spawn_times = [100, 180, 0]
    spawn_time_labels = ['100s', '180s', '$\\infty$']
    
    # Define evaporation values and their labels
    evap_values = ['50', '100', '150', '$\\infty$']
    
    # Define agent counts to show
    # agent_counts = [2, 4, 6, 10, 15]
    agent_counts = [2, 15]
    
    # Set labels based on metric
    if metric == 'f1':
        metric_name = ' f1_score'
        y_label = 'F1 Score (%)'
    elif metric == 'cpm':
        metric_name = ' coverage'
        y_label = '$CP_m$ (%)'
    elif metric == 'acp':
        metric_name = ''
        y_label = '$ACP$ (%)'
    
    # Calculate dimensions for the facet grid
    n_agents = len(agent_counts)
    n_spawn = len(spawn_times)
    
    # Set up a grid of subplots - one row for each agent count
    fig, axes = plt.subplots(n_agents, 1, figsize=(14, 4*n_agents), sharex=True)
    
    # Set colors for spawn times
    colors = plt.cm.viridis(np.linspace(0, 1, n_spawn))
    colors = [lighten_color(c, lighten_factor) for c in colors]    

    # Bar width and spacing
    group_width = 0.8  # Width of entire group for one evap value
    alg_width = group_width / 2  # Width for one algorithm group
    bar_width = alg_width / n_spawn  # Width for individual bars
    
    # Collect all data first
    data = {}
    for spawn_time in spawn_times:
        if metric == 'f1':
            file_path_CLARE = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
            file_path_Base = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/dynam_and_evap_cohesionalignment/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
        elif metric == 'acp':
            file_path_CLARE = f'results_CLARE/coverage_plots/dynam_and_evap/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
            file_path_Base = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/coverage_plots/dynam_and_evap_cohesionalignment/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        elif metric == 'cpm':
            file_path_CLARE = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
            file_path_Base = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/dynam_and_evap_cohesionalignment/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        
        # Load data for BICLARE-PP
        if os.path.exists(file_path_CLARE):
            if spawn_time not in data:
                data[spawn_time] = {}
            data[spawn_time]['BICLARE-PP'] = pd.read_csv(file_path_CLARE)
        else:
            print(f"File not found: {file_path_CLARE}")
        
        # Load data for FSP-G
        if os.path.exists(file_path_Base):
            if spawn_time not in data:
                data[spawn_time] = {}
            data[spawn_time]['FSP-G'] = pd.read_csv(file_path_Base)
        else:
            print(f"File not found: {file_path_Base}")
    
    # For each agent count (row in the facet)
    for row, agent_count in enumerate(agent_counts):
        ax = axes[row]
    
        # For each evaporation value (group on x-axis)
        for e, evap in enumerate(evap_values):
            # For each spawn time
            for s, spawn_time in enumerate(spawn_times):
                # Position calculation - base position for this evap value
                x_base = e
                
                # BICLARE-PP bars
                if spawn_time in data and 'BICLARE-PP' in data[spawn_time]:
                    df = data[spawn_time]['BICLARE-PP']
                    agent_row = df[df['n_agents'] == f'{agent_count}_agents']
                    
                    if not agent_row.empty:
                        col_name = f'evap {evap}{metric_name}'
                        std_col_name = f'std dev evap {evap}{metric_name}'
                        
                        if col_name in agent_row.columns:
                            value = agent_row[col_name].values[0]
                            error = agent_row[std_col_name].values[0] if std_col_name in agent_row.columns else 0
                            
                            # Left side bar position
                            x_pos = x_base - bar_width * (n_spawn - s) + bar_width/2
                            
                            # Plot BICLARE-PP bar
                            ax.bar(
                                x_pos, 
                                value, 
                                width=bar_width*0.9, 
                                yerr=error, 
                                capsize=3, 
                                color=colors[s],
                                alpha=0.8,
                                label=f'BICLARE-PP ({spawn_time_labels[s]})' if row == 0 and e == 0 else None
                            )
                
                # FSP-G bars
                if spawn_time in data and 'FSP-G' in data[spawn_time]:
                    df = data[spawn_time]['FSP-G']
                    fsp_agent_row = df[df['n_agents'] == f'{agent_count}_agents']
                    
                    if not fsp_agent_row.empty:
                        col_name = f'evap {evap}{metric_name}'
                        std_col_name = f'std dev evap {evap}{metric_name}'
                        
                        if col_name in fsp_agent_row.columns:
                            value = fsp_agent_row[col_name].values[0]
                            error = fsp_agent_row[std_col_name].values[0] if std_col_name in fsp_agent_row.columns else 0
                            
                            # Right side bar position
                            x_pos = x_base + bar_width * s + bar_width/2
                            
                            # Plot FSP-G bar
                            ax.bar(
                                x_pos,
                                value,
                                width=bar_width*0.9,
                                yerr=error,
                                capsize=3,
                                color=colors[s],
                                edgecolor='black',
                                hatch='////',
                                alpha=0.8,
                                label=f'FSP-G ({spawn_time_labels[s]})' if row == 0 and e == 0 and s == 0 else None
                            )
        
        # Customize row subplot
        ax.set_title(f'{agent_count} Agents', fontsize=25)
        ax.set_ylabel(y_label, fontsize=25)
        
        # Add grid (optional)
        ax.grid(True, linestyle='--', alpha=0.6, axis='y')
        
        # Set y-axis limits based on metric
        if metric == 'f1':
            ax.set_ylim(94, 100)
        elif metric == 'cpm':
            ax.set_ylim(25, 106)
        elif metric == 'acp':
            ax.set_ylim(20, 106)
        
        # Format y-tick labels
        if metric == 'f1':
            ax.set_yticklabels([f"{y}" for y in ax.get_yticks()], fontsize=22)
        else:
            ax.set_yticklabels([f"{int(y)}" for y in ax.get_yticks()], fontsize=22)
    
    # Set the x-tick positions and labels (only for the bottom subplot)
    axes[-1].set_xticks(np.arange(len(evap_values)))
    axes[-1].set_xticklabels([f'{e}s' if e!='$\\infty$' else f'{e}' for e in evap_values], fontsize=22)
    axes[-1].set_xlabel('$T_{{\\varphi}}$', fontsize=25)
    
    # Create proper legend
    # Spawn time color handles
    spawn_color_handles = [plt.Rectangle((0,0),1,1, color=colors[s]) for s in range(n_spawn)]
    spawn_labels = spawn_time_labels
    
    # Algorithm handles
    # Create visible legend handles for algorithms
    biclare_handle = plt.Rectangle((0,0),1,1, facecolor='white', edgecolor='black')
    fspg_handle = plt.Rectangle((0,0),1,1, facecolor='white', edgecolor='black', hatch='////')
    
    # Create separate legends with better positioning control using fig.legend
    # 1. Spawn time legend (at top left)
    l1 = fig.legend(spawn_color_handles, spawn_labels, 
                   title='$T_{{spawn}}$', 
                   loc='upper left',
                   bbox_to_anchor=(0.10, 0.98),
                   ncol=len(spawn_time_labels),  # Make legend horizontal
                   fontsize=22, title_fontsize=22)
    
    # 2. Algorithm legend (at top right)
    l2 = fig.legend([biclare_handle, fspg_handle], ['BICLARE-PP', 'FSP-G'],
                  title='Algorithm', 
                  loc='upper right',
                  bbox_to_anchor=(0.88, 0.99),
                  fontsize=22, title_fontsize=22)
    # Adjust layout to make room for legend
    # plt.tight_layout(rect=[0, 0.02, 0.85, 0.98])
    plt.tight_layout(rect=[0, 0.02, 1, 0.84])

    
    # Create directory if it doesn't exist
    output_dir = f'results_CLARE/accuracy_plots/dynam_and_evap/plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save the plot
    # plt.savefig(f'{output_dir}/{metric}_facet_comparison_all_algorithms.png', dpi=300)
    plt.savefig(f'{output_dir}/{metric}_facet_comparison_all_algorithms.pdf', dpi=300, bbox_inches='tight', format='pdf')
    plt.close()
    
    print(f"Faceted plot with algorithm comparison saved to {output_dir}/{metric}_facet_comparison_all_algorithms.png")

# def plot_spawn_time_evap_grouped_barplot(metric='f1', n_agents=10):
#     """
#     Creates a grouped bar chart showing all spawn times and evaporation values for a specific agent count.
    
#     Parameters:
#     metric (str): 'f1', 'cpm', or 'acp'
#     n_agents (int): Number of agents to display data for
#     """
#     # Define spawn times and their labels
#     spawn_times = [100, 180, 0]
#     spawn_time_labels = ['100s', '180s', '$\\infty$']
    
#     # Define evaporation values
#     evap_values = ['50', '100', '150', '$\\infty$']
    
#     # Set up figure
#     fig, ax = plt.subplots(figsize=(12, 8))
    
#     # Set labels based on metric
#     if metric == 'f1':
#         metric_name = 'f1_score'
#         y_label = 'F1 Score (%)'
#         title = f'F1 Score Comparison ({n_agents} agents)'
#     elif metric == 'cpm':
#         metric_name = 'coverage'
#         y_label = '$CP_m$ (%)'
#         title = f'Coverage ($CP_m$) Comparison ({n_agents} agents)'
#     elif metric == 'acp':
#         metric_name = 'coverage'
#         y_label = '$ACP$ (%)'
#         title = f'Area Coverage ($ACP$) Comparison ({n_agents} agents)'
    
#     # Width of a group and bar
#     group_width = 0.8
#     bar_width = group_width / len(evap_values)
    
#     # Colors for evaporation values
#     colors = plt.cm.tab10(np.linspace(0, 1, len(evap_values)))
#     colors = [lighten_color(c, lighten_factor) for c in colors]    

    
#     # Collect all the data
#     all_values = {}
#     all_errors = {}
#     for spawn_time in spawn_times:
#         all_values[spawn_time] = []
#         all_errors[spawn_time] = []
        
#         if metric == 'f1':
#             file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_f1_per_config_spawn_time_{spawn_time}.csv'
#         elif metric == 'acp':
#             file_path = f'results_CLARE/coverage_plots/dynam_and_evap/evap_averaged_metrics_ACP_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
#         elif metric == 'cpm':
#             file_path = f'results_CLARE/accuracy_plots/dynam_and_evap/evap_averaged_metrics_cp_m_per_agents_noise_0_per_config_spawn_time_{spawn_time}.csv'
        
#         if os.path.exists(file_path):
#             df = pd.read_csv(file_path)
#             agent_row = df[df['n_agents'] == f'{n_agents}_agents']
            
#             if not agent_row.empty:
#                 for evap in evap_values:
#                     col_name = f'evap {evap} {metric_name}'
#                     std_col_name = f'std dev evap {evap} {metric_name}'
                    
#                     if col_name in agent_row.columns:
#                         all_values[spawn_time].append(agent_row[col_name].values[0])
#                         if std_col_name in agent_row.columns:
#                             all_errors[spawn_time].append(agent_row[std_col_name].values[0])
#                         else:
#                             all_errors[spawn_time].append(0)
#                     else:
#                         all_values[spawn_time].append(np.nan)
#                         all_errors[spawn_time].append(0)
#             else:
#                 for _ in evap_values:
#                     all_values[spawn_time].append(np.nan)
#                     all_errors[spawn_time].append(0)
#         else:
#             print(f"File not found: {file_path}")
#             for _ in evap_values:
#                 all_values[spawn_time].append(np.nan)
#                 all_errors[spawn_time].append(0)
    
#     # Plot grouped bars
#     x = np.arange(len(spawn_times))  # Group positions
    
#     for i, evap in enumerate(evap_values):
#         values = [all_values[spawn_time][i] if spawn_time in all_values else np.nan for spawn_time in spawn_times]
#         errors = [all_errors[spawn_time][i] if spawn_time in all_errors else 0 for spawn_time in spawn_times]
        
#         # Calculate positions for this set of bars
#         positions = x + (i - len(evap_values)/2 + 0.5) * bar_width
        
#         ax.bar(
#             positions, 
#             values, 
#             width=bar_width * 0.9, 
#             yerr=errors, 
#             capsize=4, 
#             color=colors[i], 
#             label=f'Evap Time = {evap}',
#             alpha=0.8
#         )
    
#     # Compare with FSP-G if available (only for metric='f1')
#     if metric == 'f1':
#         try:
#             fsp_values = []
#             fsp_errors = []
            
#             for spawn_time in spawn_times:
#                 fsp_file_path = f'../Thesis_ARGoS2/results_Base_cohesion_alignment/accuracy_plots/dynam_and_evap/fsp_metrics_spawn_time_{spawn_time}.csv'
                
#                 if os.path.exists(fsp_file_path):
#                     fsp_df = pd.read_csv(fsp_file_path)
#                     fsp_agent_row = fsp_df[fsp_df['n_agents'] == f'{n_agents}_agents']
                    
#                     if not fsp_agent_row.empty and 'f1_score' in fsp_agent_row.columns:
#                         fsp_values.append(fsp_agent_row['f1_score'].values[0])
                        
#                         if 'std dev f1_score' in fsp_agent_row.columns:
#                             fsp_errors.append(fsp_agent_row['std dev f1_score'].values[0])
#                         else:
#                             fsp_errors.append(0)
#                     else:
#                         fsp_values.append(np.nan)
#                         fsp_errors.append(0)
#                 else:
#                     fsp_values.append(np.nan)
#                     fsp_errors.append(0)
            
#             # Plot FSP-G as a line with markers
#             ax.plot(x, fsp_values, 'k--', marker='s', linewidth=2, label='FSP-G')
#             for i, (val, err) in enumerate(zip(fsp_values, fsp_errors)):
#                 if not np.isnan(val):
#                     ax.errorbar(x[i], val, yerr=err, fmt='none', ecolor='black', capsize=4)
                    
#         except Exception as e:
#             print(f"Could not load FSP-G data: {e}")
    
#     # Set x-axis ticks and labels
#     ax.set_xticks(x)
#     ax.set_xticklabels(spawn_time_labels, fontsize=14)
    
#     # Set y-axis limits based on metric
#     if metric == 'f1':
#         ax.set_ylim(96, 99)
#     elif metric in ['cpm', 'acp']:
#         ax.set_ylim(25, 106)
    
#     # Set axis labels and title
#     ax.set_xlabel('Spawn Time', fontsize=18)
#     ax.set_ylabel(y_label, fontsize=18)
#     # ax.set_title(title, fontsize=18)
    
#     # Add grid and legend
#     # ax.grid(True, linestyle='--', alpha=0.6, axis='y')
#     ax.legend(fontsize=18, loc='best')
    
#     # Add y-tick labels
#     ax.set_yticklabels([f"{int(y)}%" for y in ax.get_yticks()], fontsize=16)
    
#     plt.tight_layout()
    
#     # Create directory if it doesn't exist
#     output_dir = f'results_CLARE/accuracy_plots/dynam_and_evap/plots'
#     if not os.path.exists(output_dir):
#         os.makedirs(output_dir)
    
#     # Save the plot
#     plt.savefig(f'{output_dir}/{metric}_grouped_barplot_{n_agents}_agents.png', dpi=300)
#     plt.close()
    
#     print(f"Grouped bar plot saved to {output_dir}/{metric}_grouped_barplot_{n_agents}_agents.png")



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

# # Generate all plots
# for n_agents in [2,4,6,10,15]:  # Choose representative agent counts
#     # plot_spawn_time_evap_comparison(metric='f1', n_agents=n_agents)
#     # plot_spawn_time_evap_comparison(metric='cpm', n_agents=n_agents)
#     # plot_spawn_time_evap_comparison(metric='acp', n_agents=n_agents)
#     plot_spawn_time_evap_grouped_barplot(metric='f1', n_agents=n_agents)
#     plot_spawn_time_evap_grouped_barplot(metric='cpm', n_agents=n_agents)
#     # plot_spawn_time_evap_grouped_barplot(metric='acp', n_agents=n_agents)

# for spawn_time in [100, 180, 0]:
#     plot_evap_comparison_per_spawn_time(metric='f1', spawn_time=spawn_time)
#     plot_evap_comparison_per_spawn_time(metric='cpm', spawn_time=spawn_time)
#     plot_evap_comparison_per_spawn_time(metric='acp', spawn_time=spawn_time)

# for evap in ['50', '100', '150', '$\\infty$']:
#     plot_spawn_time_comparison(metric='f1', evap_value=evap)
#     plot_spawn_time_comparison(metric='cpm', evap_value=evap)
#     plot_spawn_time_comparison(metric='acp', evap_value=evap)

# plot_spawn_time_comparison_facet(metric='f1')
# plot_spawn_time_comparison_facet(metric='cpm')
# plot_spawn_time_comparison_facet(metric='acp')

plot_spawn_time_comparison_facet_comp_algs(metric='f1')
plot_spawn_time_comparison_facet_comp_algs(metric='cpm')
plot_spawn_time_comparison_facet_comp_algs(metric='acp')

