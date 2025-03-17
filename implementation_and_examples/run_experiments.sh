#!/bin/bash

cd build || exit
make
cd ..

# Directory containing ARGoS3 experiment files
EXPERIMENT_DIR="./experiments"
CONFIG_DIR="./agent_implementation/configs"
LOG_DIR="./logs"
ARGOSEXEC="argos3"

# Ensure log directory existsd
mkdir -p "$LOG_DIR"

# List of experiment files (modify as needed)
#EXPERIMENTS=("house.argos" "house_tilted.argos" "office.argos" "office_tilted.argos" "museum.argos" "museum_tilted.argos")
EXPERIMENTS=("house.argos")
#CONFIGS=("config__alignment0_1__cohesion__0.yaml" "config__alignment0_1__cohesion__0_1.yaml" "config__alignment0__cohesion__0.yaml" "config__alignment0__cohesion__0_1.yaml")
#CONFIGS=("config_bigger_safety_n_1.yaml" "config_bigger_safety_range.yaml" "config_bigger_safety_n_3.yaml")
#CONFIGS=("n_3_m_2_5_cellratio0_75_noise.yaml" "n_3_m_2_5_cellratio0_75_noise_agent_avoidance_0_5.yaml" "n_3_m_2_5_cellratio0_75_noise_object_safety_0_3.yaml")
#CONFIGS=("p_sensor_1.yaml" "p_sensor_0_9.yaml" "p_sensor_0_75.yaml" "p_sensor_0_5.yaml")
CONFIGS=("p_sensor_0_5_no_doubt.yaml")



PARALLEL_JOBS=5
declare -A pids  # Associative array to store PIDs and their related info

N_AGENTS=15
MAX_AGENTS=15
MIN_AGENTS=15

N_REPEATED_EXPERIMENTS=1

for r in $(seq 1 $((N_REPEATED_EXPERIMENTS))); do
  echo "Running repeated experiment $r"
  SEED=$r
  echo "Seed: $SEED"
  export SEED

  # Iterate over each experiment file
  for EXPERIMENT in "${EXPERIMENTS[@]}"; do
      EXP_PATH="$EXPERIMENT_DIR/$EXPERIMENT"
      export EXPERIMENT


      for CONFIG_FILE in "${CONFIGS[@]}"; do
          CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
  #        export CONFIG_FILE

          sed "s|{{CONFIG_PATH}}|${CONFIG_PATH}|g" "$EXP_PATH" > "temp_${CONFIG_FILE%.yaml}_S${SEED}_${EXPERIMENT}"

          #Copy the config file into the metric path
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



          for remove_agents in $(seq $((N_AGENTS-MAX_AGENTS)) $((N_AGENTS-MIN_AGENTS))); do
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
              for AVERAGE_INTER_SPAWN_TIME in {0..0..0} # one step
              do
                echo "Number of running jobs: $(pgrep -c -u $USER $ARGOSEXEC)"
                #Wait if we have more than PARALLEL_JOBS jobs running
                while [ $(pgrep -c -u $USER $ARGOSEXEC) -ge $PARALLEL_JOBS ]; do
                    sleep 1
                done
                export AVERAGE_INTER_SPAWN_TIME

                METRIC_PATH="experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/spawn_time_${AVERAGE_INTER_SPAWN_TIME}/${REMAINING_AGENTS}_agents/S${SEED}"
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
                echo "ARGoS3 process $PID started"
                pids[$PID]="$SEED|$EXPERIMENT|$CONFIG_FILE|$AVERAGE_INTER_SPAWN_TIME|$REMAINING_AGENTS|$LOGFILE"
              done
        done

      done

  done
done

# Wait for all processes to finish
for pid in "${!pids[@]}"; do
    wait $pid
    exit_code=$?
    echo "ARGoS3 process $pid has exited with status $exit_code"
    IFS='|' read -r SEED EXPERIMENT CONFIG_FILE AVERAGE_INTER_SPAWN_TIME REMAINING_AGENTS LOGFILE <<< "${pids[$pid]}"
    if [ $exit_code -eq 0 ]; then
        echo "Experiment '$EXPERIMENT' with config '$CONFIG_FILE' and seed $SEED with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME and ${REMAINING_AGENTS} agents finished successfully."
    else
        echo "Experiment '$EXPERIMENT' with config '$CONFIG_FILE' and seed $SEED with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME and ${REMAINING_AGENTS} agents failed. Check log: $LOGFILE" >&2
    fi
done

#rm temp_*

echo "All experiments finished."
