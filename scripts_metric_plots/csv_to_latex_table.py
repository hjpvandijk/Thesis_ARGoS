import os
import pandas as pd

def sorting_key(filename):
    if not filename.endswith(".csv"):
        return 999
    # Extract the part of the filename after the first underscore
    # and convert it to lowercase for case-insensitive sorting
    if "house_tilted" in filename:
        return 2
    if "house" in filename:
        return 1
    if "office_tilted" in filename:
        return 4
    if "office" in filename:
        return 3
    if "museum_tilted" in filename:
        return 6
    if "museum" in filename:
        return 5
    else:
        print("ERROR: Unknown filename format:", filename)
        return 0

def csv_to_latex_table(directory, type, exps, noiseval, spawn_timeval, n_columns, multicols, max_vertical_tables):
    # Ensure the directory exists
    if not os.path.isdir(directory):
        print(f"Error: Directory '{directory}' does not exist.")
        return

    # Initialize a list to store sub-tables
    sub_tables = []

    # Iterate over all files in the directory
    for filename in sorted(os.listdir(directory), key=sorting_key):
        if filename.endswith(".csv"):
            if type not in filename:
                continue
            filepath = os.path.join(directory, filename)

            underscores_in_type = type.count("_")
            map_name = filename.split("_")[underscores_in_type+1]
            if "tilted" in filename:
                map_name_parts = filename.split("_")[underscores_in_type+1:underscores_in_type + 3]
                map_name = "_".join(map_name_parts)
            map_name = map_name.replace("_", "\_")

            splitted = filename.split('_')
            spawn_time = int(splitted[-1].split('.')[0])
            if spawn_time != spawn_timeval:
                continue
            noise = float(splitted[-4])
            #if splitted[-2] is a number, prepend it to noise with a decimal
            if noise != noiseval:
                continue

        
            # Read the CSV file into a DataFrame
            try:
                df = pd.read_csv(filepath)
                #replace column name n_agents with '$N_A$'
                df.rename(columns={'n_agents': '$N_A$'}, inplace=True)
                #in the column '$N_A$' for each value remove '_agents'
                df['$N_A$'] = df['$N_A$'].str.replace('_agents', '', regex=False)

                #get all columns with 'mean' in the name
                mean_columns = [col for col in df.columns if 'mean' in col]
                #remove 'mean' from the column names
                datatypes = [col.replace('mean ', '') for col in mean_columns]

            except Exception as e:
                print(f"Error reading {filename}: {e}")
                continue

            for datatype in datatypes:

                #only include the columns that contain the datatype, and tne column '$N_A$'
                df_dt = df[[col for col in df.columns if datatype in col or col == '$N_A$']]
                #remove the ' {datatype}' from the column names
                df_dt.rename(columns=lambda col: col.replace(f' {datatype}', '') if f' {datatype}' in col else col, inplace=True)

                # Replace any column name containing 'standard deviation' with a sigma
                df_dt.rename(columns=lambda col: col.replace('Standard Deviation', '$\\sigma$') if 'Standard Deviation' in col else col, inplace=True)

                df_dt.rename(columns=lambda col: col.replace('mean', '$\\mu$') if 'mean' in col else col, inplace=True)
                #round all values to one decimal place
                df_dt = df_dt.round(2)
            
                # Convert the DataFrame to a LaTeX table
                latex_table = df_dt.to_latex(index=False, escape=False)
                # print(latex_table)

                caption = f"\\textsc{{{map_name}}}"
                label = os.path.splitext(filename)[0]
                if datatype != 'mean':
                    datatype_with_capitalization = datatype.capitalize()
                    if datatype == 'free':
                        datatype_with_capitalization = '$AAC_F'
                    elif datatype == 'occupied':
                        datatype_with_capitalization = '$AAC_O'
                    elif datatype == 'all':
                        datatype_with_capitalization = '$AAC_A'
                    caption = f"{{{datatype_with_capitalization}}} for " + caption
                    label = f"{{{datatype}}}_" + label
                
                table_size = 1/n_columns
                if n_columns > 1:
                    table_size -= 0.02

                if multicols:
                    tablesizestring = "\\linewidth"
                else:
                    tablesizestring = f"{table_size}\\textwidth"
                # Create a sub-table with a caption
                sub_table = f"""
                \\begin{{subtable}}[t]{{{tablesizestring}}}
                \\centering
                {latex_table}
                \\caption{{{caption}}}
                \\label{{subtab:{label}}}
                \\end{{subtable}}
                """                                     
                sub_tables.append(sub_table)

                
    
    typecaption = type
    if type == "CP(T_end)":
        typecaption = "$CP(T_{{end}})$"
    elif type == "AC(T_end)":
        typecaption = "$AC(T_{{end}})$"
    elif type == "traveled_path":
        typecaption = "$L_{{traveled}}$"

    exps_caption = exps
    if exps == "fsr_mfr_mrl":
        exps_caption = "computation-saving parameters"

    # Combine all sub-tables into a main table
    if sub_tables:
        # print(typecaption)
        # print(exps_caption)
        # print("\\hfill".join(sub_tables))
        # return

        midpart = ""
        for i, sub_table in enumerate(sub_tables):
            # Add a new line after every n_columns sub-tables
            if (i+1) % (max_vertical_tables*n_columns) != 0 and  i != len(sub_tables) - 1:
                
                midpart += sub_table + "\\smallskip"
            elif (i+1) % (max_vertical_tables*n_columns) == 0 and i != 0 and i != len(sub_tables) - 1:
                midpart += sub_table
                # Create a new table for the next set of sub-tables
                if multicols:
                    midpart += f"""
                    \\end{{multicols}}
                    \\end{{table}}
                    \\clearpage
                    \\begin{{table}}[ht]
                    \\ContinuedFloat
                    \\centering
                    \\begin{{multicols}}{{{n_columns}}}
                    """
                else:
                    midpart += f"""
                    \\end{{table}}
                    \\clearpage
                    \\begin{{table}}[ht]
                    \\ContinuedFloat
                    \\centering
                    """
            elif i == len(sub_tables) - 1:
                midpart += sub_table

        if multicols:
            main_table = f"""
            \\begin{{table}}[ht]
            \\centering
            \\begin{{multicols}}{{{n_columns}}}
            """
            main_table += midpart
            main_table += f"""
            \\end{{multicols}}
            \\caption{{{typecaption} for {exps_caption} with noise {noiseval} and spawn time {spawn_timeval}}}
            \\label{{tab:{type.replace(" ", "_")}_{exps_caption.replace(" ", "_")}}}
            \\end{{table}}
            """
        else:
            main_table = f"""
            \\begin{{table}}[ht]
            \\centering
            {midpart}
            \\caption{{{typecaption} for {exps_caption} with noise {noiseval} and spawn time {spawn_timeval}}}
            \\label{{tab:{type.replace(" ", "_")}_{exps_caption.replace(" ", "_")}}}
            \\end{{table}}
            """
        
        # Create the directory if it doesn't exist
        os.makedirs(os.path.join(directory, "latex_tables"), exist_ok=True)

        # Save the main table to a .tex file
        output_filepath = os.path.join(directory, f"latex_tables/{type}_{exps_caption}_noise_{noiseval}_spawn_time_{spawn_timeval}.tex")
        try:
            with open(output_filepath, "w") as tex_file:
                tex_file.write(main_table)
            print(f"Generated main LaTeX table -> {output_filepath}")
        except Exception as e:
            print(f"Error writing to {output_filepath}: {e}")
    else:
        print(f"No CSV files found in the directory: {directory} with type {type} and noise {noiseval} and spawn time {spawn_timeval}")

if __name__ == "__main__":
    # Specify the directory containing the CSV files
    input_directory = "results/coverage_plots/fsr_mfr_mrl/csv_tables"
    types = ["ACP", "CP(T_end)"]
    n_columns = 2
    multicols = False
    max_vertical_tables = 6

    # input_directory = "accuracy_plots/fsr_mfr_mrl/csv_tables"
    # types = ["accuracy"]
    # n_columns = 2
    # multicols = True
    # max_vertical_tables = 6

    # input_directory = "traveled_path/fsr_mfr_mrl/csv_tables"
    # types = ["traveled_path"]
    # n_columns = 3
    # multicols = False
    # max_vertical_tables = 99999
    noises = [0,0.5,1,1.5]
    spawn_times = [0,100,180]

    for type in types:
        for noise in noises:
            for spawn_time in spawn_times:
                csv_to_latex_table(input_directory, type, "fsr_mfr_mrl", noise, spawn_time, n_columns, multicols, max_vertical_tables)