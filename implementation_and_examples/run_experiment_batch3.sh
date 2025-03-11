#!/bin/bash

cd build || exit
make
cd ..

# Directory containing ARGoS3 experiment files
EXPERIMENT_DIR="./experiments"
CONFIG_DIR="./agent_implementation/configs/noise"
OTHER_CONFIG_DIRS=() #("./agent_implementation/configs/comm_range_and_loss")
LOG_DIR="./logs"
ARGOSEXEC="argos3"

# Ensure log directory existsd
mkdir -p "$LOG_DIR"

# List of experiment files (modify as needed)
#EXPERIMENTS=("house.argos" "house_tilted.argos" "office.argos" "office_tilted.argos" "museum.argos" "museum_tilted.argos")
EXPERIMENTS=("house.argos" "house_tilted.argos" "office.argos" "office_tilted.argos")
#EXPERIMENTS=("museum_tilted.argos")
#CONFIGS=("config__alignment0_1__cohesion__0.yaml" "config__alignment0_1__cohesion__0_1.yaml" "config__alignment0__cohesion__0.yaml" "config__alignment0__cohesion__0_1.yaml")
#CONFIGS=("config_bigger_safety_n_1.yaml" "config_bigger_safety_range.yaml" "config_bigger_safety_n_3.yaml")
#CONFIGS=("n_3_m_2_5_cellratio0_75_noise.yaml" "n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5.yaml" "n_3_m_2_5_cellratio0_75_noise_object_safety_0_3.yaml")
#CONFIGS=("p_sensor_1.yaml")
CONFIGS=(
        "end_time_{END_TIME}_noise_0_wifi_range_15_message_loss_probability_0_frontier_search_radius_99999_evaporation_time_100_max_frontier_cells_99999_max_route_length_99999.yaml"
        "end_time_{END_TIME}_noise_0_5_wifi_range_15_message_loss_probability_0_frontier_search_radius_99999_evaporation_time_100_max_frontier_cells_99999_max_route_length_99999.yaml"
        "end_time_{END_TIME}_noise_1_wifi_range_15_message_loss_probability_0_frontier_search_radius_99999_evaporation_time_100_max_frontier_cells_99999_max_route_length_99999.yaml"
        "end_time_{END_TIME}_noise_1_5_wifi_range_15_message_loss_probability_0_frontier_search_radius_99999_evaporation_time_100_max_frontier_cells_99999_max_route_length_99999.yaml"
        )


PARALLEL_JOBS=6
declare -A pids  # Associative array to store PIDs and their related info

N_AGENTS=15
#MAX_AGENTS=15
#MIN_AGENTS=15

AGENT_CONFIGS=(15 10 6 4 2)

AVERAGE_INTER_SPAWN_TIMES=(0 100 180)

N_REPEATED_EXPERIMENTS=3

n_total_experiments_to_run=$((N_REPEATED_EXPERIMENTS*${#EXPERIMENTS[@]}*${#CONFIGS[@]}*${#AGENT_CONFIGS[@]}*${#AVERAGE_INTER_SPAWN_TIMES[@]}))
n_experiments_started=0
n_experiments_already_exist=0
n_successful_experiments=0
n_failed_experiments=0

for r in $(seq 1 $((N_REPEATED_EXPERIMENTS))); do
#  echo "Running repeated experiment $r"
  SEED=$r
#  echo "Seed: $SEED"
  export SEED

  # Iterate over each experiment file
  for EXPERIMENT in "${EXPERIMENTS[@]}"; do
      EXP_PATH="$EXPERIMENT_DIR/$EXPERIMENT"
      export EXPERIMENT


      for CONFIG_FILE in "${CONFIGS[@]}"; do
          CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
          #if 'house' in experiment name, replace '{END_TIME} with 400 in config file name
          #if 'office', replace with 600
          #if 'museum', replace with 1000
          if [[ $EXPERIMENT == *"house"* ]]; then
            CONFIG_FILE="${CONFIG_FILE//\{END_TIME\}/400}"
            CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
          elif [[ $EXPERIMENT == *"office"* ]]; then
            CONFIG_FILE="${CONFIG_FILE//\{END_TIME\}/600}"
            CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
          elif [[ $EXPERIMENT == *"museum"* ]]; then
            CONFIG_FILE="${CONFIG_FILE//\{END_TIME\}/1000}"
            CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
          else
            echo "Error: Experiment '$EXPERIMENT' not recognized!" >&2
            continue
          fi

          #for all other config directories, check if the config already exists there. If so skip this experiment
          for OTHER_CONFIG_DIR in "${OTHER_CONFIG_DIRS[@]}"; do
            if [ -f "$OTHER_CONFIG_DIR/$CONFIG_FILE" ]; then
              echo "Config file already exists in $OTHER_CONFIG_DIR: $CONFIG_FILE"
              #add all experiments that we would run to already_exists
              n_experiments_already_exist=$((n_experiments_already_exist+${#AGENT_CONFIGS[@]}*${#AVERAGE_INTER_SPAWN_TIMES[@]}))
              continue 2 #skip this config file
            fi
          done


          sed "s|{{CONFIG_PATH}}|${CONFIG_PATH}|g" "$EXP_PATH" > "temp_${CONFIG_FILE%.yaml}_S${SEED}_${EXPERIMENT}"

          #Copy the config file into the metric path
          mkdir -p "experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/"
          cp "$CONFIG_PATH" "experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/"


          for i in $(seq 1 $((N_AGENTS))); do
            ORIENTATION=$(shuf -i 0-360 -n 1 --random-source=<(openssl enc -aes-256-ctr -pbkdf2 -pass pass:$((SEED * i)) -nosalt < /dev/zero 2>/dev/null))
            #if 'tilted' in experiment name, add -20 degrees to the orientation
            #So that the tilted versions have relatively the same orientation as the non-tilted versions
            if [[ $EXPERIMENT == *"tilted"* ]]; then
              ORIENTATION=$((ORIENTATION+20))
            fi

            AGENT_ID="PIPUCK$i"

            sed -i "s/{{${AGENT_ID}_ORIENTATION}}/${ORIENTATION}/" "temp_${CONFIG_FILE%.yaml}_S${SEED}_${EXPERIMENT}"
#            echo "Agent $AGENT_ID orientation: $ORIENTATION"
          done



          for agent_config in "${AGENT_CONFIGS[@]}"; do
            remove_agents=$((N_AGENTS-agent_config))
            REMAINING_AGENTS=$((N_AGENTS-remove_agents))


              # Remove agents from the experiment file
              if [ $remove_agents -ne 0 ]; then
                  for i in $(seq 1 $((remove_agents))); do
                    sed -i "/<pipuck id=\"pipuck$((N_AGENTS-i+1))\"/,/<\/pipuck>/d" "temp_${CONFIG_FILE%.yaml}_S${SEED}_${EXPERIMENT}"
                  done
              fi

              # Check if the configuration file exists
              if [ ! -f "$EXP_PATH" ]; then
                  echo "Error: Experiment config '$EXP_PATH' not found!" >&2
                  continue
              fi

  #            for AVERAGE_INTER_SPAWN_TIME in {120..20..0} # for loop from 300 to 30 with step 30
              for AVERAGE_INTER_SPAWN_TIME in "${AVERAGE_INTER_SPAWN_TIMES[@]}"
              do
#                echo "Number of running jobs: $(pgrep -c -u $USER $ARGOSEXEC)"
                #Wait if we have more than PARALLEL_JOBS jobs running
                while [ $(pgrep -c -u $USER $ARGOSEXEC) -ge $PARALLEL_JOBS ]; do
                    sleep 1
                done
                export AVERAGE_INTER_SPAWN_TIME

                METRIC_PATH="experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/spawn_time_${AVERAGE_INTER_SPAWN_TIME}/${REMAINING_AGENTS}_agents/S${SEED}"
                #if it already exists, skip this experiment
                if [ -d "$METRIC_PATH" ]; then
#                  #if "certainty.csv" exists, skip this experiment
                  if [ -f "$METRIC_PATH/certainty.csv" ]; then
                    echo "Experiment already exists: $METRIC_PATH"
                    n_experiments_already_exist=$((n_experiments_already_exist+1))
                    continue
                  else
#                    if certainty.csv doesn't exist, empty the mectric path
                    rm -rf "$METRIC_PATH"/*
                  fi
                fi

                mkdir -p "$METRIC_PATH"
                export METRIC_PATH

                FRAME_PATH="experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/spawn_time_${AVERAGE_INTER_SPAWN_TIME}/${REMAINING_AGENTS}_agents/S${SEED}/frames"
                sed "s|{{FRAME_PATH}}|${FRAME_PATH}|g" "temp_${CONFIG_FILE%.yaml}_S${SEED}_${EXPERIMENT}" > "temp_${CONFIG_FILE%.yaml}_${REMAINING_AGENTS}_${AVERAGE_INTER_SPAWN_TIME}_S${SEED}_${EXPERIMENT}"

                mkdir -p "$FRAME_PATH"
                # empty the directory
                rm -rf "$FRAME_PATH"/*



                # Generate timestamp for log file
                TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
  #              LOGFILE="$LOG_DIR/${EXPERIMENT%.argos}_${CONFIG_FILE%.yaml}_spawn_time_${AVERAGE_INTER_SPAWN_TIME}_${REMAINING_AGENTS}_agents_$TIMESTAMP.log"
                LOGFILE="$METRIC_PATH/${EXPERIMENT%.argos}_S${SEED}_${CONFIG_FILE%.yaml}_spawn_time_${AVERAGE_INTER_SPAWN_TIME}_${REMAINING_AGENTS}_agents_$TIMESTAMP.log"
                echo "Running ARGoS3 with configuration: S${SEED}, $EXP_PATH and $CONFIG_PATH with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME with ${REMAINING_AGENTS} agents"



                # Run ARGoS3 and log output, and save exit code
                "$ARGOSEXEC" -n -c "temp_${CONFIG_FILE%.yaml}_${REMAINING_AGENTS}_${AVERAGE_INTER_SPAWN_TIME}_S${SEED}_${EXPERIMENT}" > "$LOGFILE" 2>&1 &
                PID=$!
#                echo "ARGoS3 process $PID started"
                pids[$PID]="$SEED|$EXPERIMENT|$CONFIG_FILE|$AVERAGE_INTER_SPAWN_TIME|$REMAINING_AGENTS|$LOGFILE"
                n_experiments_started=$((n_experiments_started+1))
                total_jobs_started_and_exist=$((n_experiments_started+n_experiments_already_exist))
                echo "Jobs started: $n_experiments_started, jobs already exist: $n_experiments_already_exist : $total_jobs_started_and_exist/$n_total_experiments_to_run"

              done
        done

#        #remove related temp files
#        rm "temp_${CONFIG_FILE%.yaml}_S${SEED}_${EXPERIMENT}"
#        for agent_config in "${AGENT_CONFIGS[@]}"; do
#          remove_agents=$((N_AGENTS-agent_config))
#          REMAINING_AGENTS=$((N_AGENTS-remove_agents))
#          for AVERAGE_INTER_SPAWN_TIME in "${AVERAGE_INTER_SPAWN_TIMES[@]}"
#          do
#            rm "temp_${CONFIG_FILE%.yaml}_${REMAINING_AGENTS}_${AVERAGE_INTER_SPAWN_TIME}_S${SEED}_${EXPERIMENT}"
#          done
#        done
      done

  done
done

failed_experiments=()

# Wait for all processes to finish
for pid in "${!pids[@]}"; do
    wait $pid
    exit_code=$?
#    echo "ARGoS3 process $pid has exited with status $exit_code"
    IFS='|' read -r SEED EXPERIMENT CONFIG_FILE AVERAGE_INTER_SPAWN_TIME REMAINING_AGENTS LOGFILE <<< "${pids[$pid]}"
    if [ $exit_code -eq 0 ]; then
        n_successful_experiments=$((n_successful_experiments+1))
#        echo "Experiment '$EXPERIMENT' with config '$CONFIG_FILE' and seed $SEED with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME and ${REMAINING_AGENTS} agents finished successfully."
    else
        n_failed_experiments=$((n_failed_experiments+1))
        failed_experiments+=("$SEED|$EXPERIMENT|$CONFIG_FILE|$AVERAGE_INTER_SPAWN_TIME|$REMAINING_AGENTS|$LOGFILE")        failed_experiments+=("${failed_experiment[@]}")
#        echo "Experiment '$EXPERIMENT' with config '$CONFIG_FILE' and seed $SEED with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME and ${REMAINING_AGENTS} agents failed. Check log: $LOGFILE" >&2
    fi
done

rm temp_*

echo "Number of successful experiments: $n_successful_experiments"
echo "Number of failed experiments: $n_failed_experiments"
echo "Number of experiments already existing: $n_experiments_already_exist"
#export failed_experiments to file
for failed_experiment in "${failed_experiments[@]}"; do
  echo "$failed_experiment" >> "failed_experiments.txt"
  #separate the failed experiments with a newline
  echo "" >> "failed_experiments.txt"
done
echo "All experiments finished."
