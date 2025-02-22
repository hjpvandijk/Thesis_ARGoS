#!/bin/bash

cd build || exit
make
cd ..

# Directory containing ARGoS3 experiment files
EXPERIMENT_DIR="./experiments"
CONFIG_DIR="./agent_implementation/configs"
LOG_DIR="./logs"
ARGOSEXEC="argos3"

# Ensure log directory exists
mkdir -p "$LOG_DIR"

# List of experiment files (modify as needed)
#EXPERIMENTS=("house.argos" "house_tilted.argos" "office.argos" "office.tilted.argos" "museum.argos" "museum.tilted.argos")
EXPERIMENTS=( "museum.argos")
CONFIGS=("config.yaml")

N_AGENTS=15
MAX_AGENTS=10
SEED=123
export SEED

# Iterate over each experiment file
for EXPERIMENT in "${EXPERIMENTS[@]}"; do
    EXP_PATH="$EXPERIMENT_DIR/$EXPERIMENT"
#    export EXPERIMENT


    for CONFIG_FILE in "${CONFIGS[@]}"; do
        CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
#        export CONFIG_FILE

        sed "s|{{CONFIG_PATH}}|${CONFIG_PATH}|g" "$EXP_PATH" > temp_experiment.argos


        for remove_agents in $(seq $((N_AGENTS-MAX_AGENTS)) $((N_AGENTS))); do
          REMAINING_AGENTS=$((N_AGENTS-remove_agents))


            # Remove agents from the experiment file
            if [ $remove_agents -ne 0 ]; then
                for i in $(seq 1 $((remove_agents))); do
                  sed -i "/<pipuck id=\"pipuck$((N_AGENTS-i+1))\"/,/<\/pipuck>/d" temp_experiment.argos
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
              export AVERAGE_INTER_SPAWN_TIME

              METRIC_PATH="experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/spawn_time_${AVERAGE_INTER_SPAWN_TIME}/${REMAINING_AGENTS}_agents"
              export METRIC_PATH
              FRAME_PATH="experiment_results/${EXPERIMENT%.argos}/${CONFIG_FILE%.yaml}/spawn_time_${AVERAGE_INTER_SPAWN_TIME}/${REMAINING_AGENTS}_agents/frames"
              sed "s|{{FRAME_PATH}}|${FRAME_PATH}|g" temp_experiment.argos > temp_experiment2.argos


              mkdir -p "$FRAME_PATH"
              # empty the directory
              rm -rf "$FRAME_PATH"/*

              # Generate timestamp for log file
              TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
              LOGFILE="$LOG_DIR/${EXPERIMENT%.argos}_${CONFIG_FILE%.yaml}_spawn_time_${AVERAGE_INTER_SPAWN_TIME}_${REMAINING_AGENTS}_agents_$TIMESTAMP.log"

              echo "Running ARGoS3 with configuration: $EXP_PATH and $CONFIG_PATH with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME with ${REMAINING_AGENTS} agents"



              # Run ARGoS3 and log output
              "$ARGOSEXEC" -n -c "temp_experiment2.argos"> "$LOGFILE" 2>&1

              # Check exit status
              if [ $? -eq 0 ]; then
                  echo "Experiment '$EXPERIMENT' with config '$CONFIG_FILE' with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME and ${REMAINING_AGENTS} agents finished successfully."
              else
                  echo "Experiment '$EXPERIMENT' with config '$CONFIG_FILE' with inter_spawn_time $AVERAGE_INTER_SPAWN_TIME and ${REMAINING_AGENTS} agents failed. Check log: $LOGFILE" >&2
              fi
            done
      done

    done

done
rm temp_experiment.argos
rm temp_experiment2.argos

echo "All experiments finished."
