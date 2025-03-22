import yaml
import os

placeholder = "implementation_and_examples/agent_implementation/configs/placeholder_config.yaml"

noise_full_position = 103.3
noise_full_position_jitter = 10
noise_full_orientation = 8
noise_full_orientation_jitter = 5
noise_distance_jitter = 10

active = "fsr_mfr_mrl"

prefix = "AAVFIX_"

def update_yaml_value(oldfile_path,file_path, key, new_value):
    """Updates a nested key in a YAML file using dot notation (e.g., 'settings.display.brightness')."""
    with open(oldfile_path, "r") as file:
        data = yaml.safe_load(file)  # Load YAML into a Python dictionary

    keys = key.split(".")  # Split the key path by dots
    d = data

    # Traverse the dictionary until the second last key
    for k in keys[:-1]:
        if k not in d or not isinstance(d[k], dict):
            raise KeyError(f"Key '{k}' not found or not a dictionary in the YAML structure.")
        d = d[k]

    # Update the final key
    if keys[-1] in d:
        d[keys[-1]] = new_value
    else:
        raise KeyError(f"Key '{keys[-1]}' not found in '{'.'.join(keys[:-1])}'.")

    with open(file_path, "w") as file:
        yaml.dump(data, file, default_flow_style=False)  # Save back to YAML

def create_configs(options, dir, prefix):
    #number of combinations
    print(len(options["mission.end_time"])*len(options["noise"])*len(options["communication.wifi_range"])*len(options["communication.message_loss_probability"])*len(options["forces.frontier_search_radius"])*len(options["forces.max_frontier_regions"])*len(options["quadtree.evaporation_time"])*len(options["control.path_planning.max_route_length"]))
    print()
    print("(")
    outer = 0
    #For all combionations of the options
    for end_time in options["mission.end_time"]:
        for noise in options["noise"]:
            for wifi_range in options["communication.wifi_range"]:
                for message_loss_probability in options["communication.message_loss_probability"]:
                    for frontier_search_radius in options["forces.frontier_search_radius"]:
                        for max_frontier_regions in options["forces.max_frontier_regions"]:
                            for evaporation_time in options["quadtree.evaporation_time"]:
                                for max_route_length in options["control.path_planning.max_route_length"]:
                                    # Construct the path to the YAML file in the current directory
                                    config_string = f"{prefix}end_time_{str(end_time).replace('.', '_')}_noise_{str(noise).replace('.', '_')}_wifi_range_{str(wifi_range).replace('.', '_')}_message_loss_probability_{str(message_loss_probability).replace('.', '_')}_frontier_search_radius_{str(frontier_search_radius).replace('.', '_')}_max_frontier_regions_{str(max_frontier_regions).replace('.', '_')}_evaporation_time_{str(evaporation_time).replace('.', '_')}_max_route_length_{str(max_route_length).replace('.', '_')}.yaml"
                                    yaml_path = os.path.join(dir, config_string)
                                    # Update the YAML file
                                    update_yaml_value(placeholder, yaml_path, "sensors.position_noise", noise_full_position*noise)
                                    update_yaml_value(yaml_path, yaml_path, "sensors.position_jitter", noise_full_position_jitter*noise)
                                    update_yaml_value(yaml_path, yaml_path, "sensors.orientation_noise", noise_full_orientation*noise)
                                    update_yaml_value(yaml_path, yaml_path, "sensors.orientation_jitter", noise_full_orientation_jitter*noise)
                                    update_yaml_value(yaml_path, yaml_path, "sensors.distance_sensor_jitter", noise_distance_jitter*noise)
                                    update_yaml_value(yaml_path, yaml_path, "sensors.distance_sensor_noise_factor", noise)
                                    
                                    update_yaml_value(yaml_path, yaml_path, "mission.end_time", end_time)
                                    update_yaml_value(yaml_path, yaml_path, "communication.wifi_range", wifi_range)
                                    update_yaml_value(yaml_path, yaml_path, "communication.message_loss_probability", message_loss_probability)
                                    update_yaml_value(yaml_path, yaml_path, "forces.frontier_search_radius", frontier_search_radius)
                                    update_yaml_value(yaml_path, yaml_path, "forces.max_frontier_regions", max_frontier_regions)
                                    update_yaml_value(yaml_path, yaml_path, "quadtree.evaporation_time", evaporation_time)
                                    update_yaml_value(yaml_path, yaml_path, "control.path_planning.max_route_length", max_route_length)
                                    
                                    if outer == 0:
                                        print(f"\"{prefix}end_time_{{END_TIME}}_noise_{str(noise).replace('.', '_')}_wifi_range_{str(wifi_range).replace('.', '_')}_message_loss_probability_{str(message_loss_probability).replace('.', '_')}_frontier_search_radius_{str(frontier_search_radius).replace('.', '_')}_max_frontier_regions_{str(max_frontier_regions).replace('.', '_')}_evaporation_time_{str(evaporation_time).replace('.', '_')}_max_route_length_{str(max_route_length).replace('.', '_')}.yaml\"")
        outer += 1

                                
    print(")")

if active == "comm":
    #Comm range and loss
    dir = "implementation_and_examples/agent_implementation/configs/comm_range_and_loss"
    #make directory
    os.makedirs(dir, exist_ok=True)

    #clear directory
    for file in os.listdir(dir):
        os.remove(os.path.join(dir, file))

    options = {
        "mission.end_time": [400, 600, 1000],
        "noise": [0, 1],
        "communication.wifi_range": [99999, 15, 10, 5],
        "communication.message_loss_probability": [0, 0.25, 0.5],
        "forces.frontier_search_radius" : [99999],
        "forces.max_frontier_regions" : [99999],
        "quadtree.evaporation_time" : [100],
        "control.path_planning.max_route_length" : [99999],
    }

    create_configs(options, dir, prefix)

elif active == "noise":

    #Noise
    dir = "implementation_and_examples/agent_implementation/configs/noise"
    #make directory
    os.makedirs(dir, exist_ok=True)

    #clear directory
    for file in os.listdir(dir):
        os.remove(os.path.join(dir, file))

    options = {
        "mission.end_time": [400, 600, 1000],
        "noise": [0, 0.5, 1, 1.5],
        "communication.wifi_range": [15],
        "communication.message_loss_probability": [0],
        "forces.frontier_search_radius" : [99999],
        "forces.max_frontier_regions" : [99999],
        "quadtree.evaporation_time" : [100],
        "control.path_planning.max_route_length" : [99999],
    }

    create_configs(options, dir, prefix)

elif active == "dynam_evap":

    #Noise
    dir = "implementation_and_examples/agent_implementation/configs/dynamic_evap"
    #make directory
    os.makedirs(dir, exist_ok=True)

    #clear directory
    for file in os.listdir(dir):
        os.remove(os.path.join(dir, file))

    options = {
        "mission.end_time": [400, 600, 1000],
        "noise": [0, 1],
        "communication.wifi_range": [15],
        "communication.message_loss_probability": [0, 0.1, 0.25, 0.5],
        "forces.frontier_search_radius" : [99999],
        "forces.max_frontier_regions" : [99999],
        "quadtree.evaporation_time" : [50, 100, 150, 200, 99999],
        "control.path_planning.max_route_length" : [99999],
    }

    create_configs(options, dir, prefix)



elif active == "fsr_mfr_mrl":

    #Noise
    dir = "implementation_and_examples/agent_implementation/configs/fsr_mfr_mrl"
    #make directory
    os.makedirs(dir, exist_ok=True)

    #clear directory
    for file in os.listdir(dir):
        os.remove(os.path.join(dir, file))

    options = {
        "mission.end_time": [400, 600, 1000],
        "noise": [0, 1],
        "communication.wifi_range": [99999],
        "communication.message_loss_probability": [0],
        "forces.frontier_search_radius" : [99999, 15, 5],
        "forces.max_frontier_regions" : [99999, 20],
        "quadtree.evaporation_time" : [100],
        "control.path_planning.max_route_length" : [99999, 30],
    }

    create_configs(options, dir, prefix)

elif active == "dynam_and_evap":

    #Noise
    dir = "implementation_and_examples/agent_implementation/configs/dynam_and_evap"
    #make directory
    os.makedirs(dir, exist_ok=True)

    #clear directory
    for file in os.listdir(dir):
        os.remove(os.path.join(dir, file))

    options = {
        "mission.end_time": [400, 600, 1000],
        "noise": [0, 1],
        "communication.wifi_range": [15],
        "communication.message_loss_probability": [0],
        "forces.frontier_search_radius" : [99999],
        "forces.max_frontier_regions" : [99999],
        "quadtree.evaporation_time" : [50, 100, 150, 200, 99999],
        "control.path_planning.max_route_length" : [99999],
    }

    create_configs(options, dir, prefix)

elif active == "try":

    #Noise
    dir = "implementation_and_examples/agent_implementation/configs/try"
    #make directory
    os.makedirs(dir, exist_ok=True)

    #clear directory
    for file in os.listdir(dir):
        os.remove(os.path.join(dir, file))

    options = {
        "mission.end_time": [400, 600, 1000],
        "noise": [0],
        "communication.wifi_range": [15],
        "communication.message_loss_probability": [0],
        "forces.frontier_search_radius" : [99999],
        "forces.max_frontier_regions" : [99999],
        "quadtree.evaporation_time" : [100],
    }

    create_configs(options, dir, prefix)