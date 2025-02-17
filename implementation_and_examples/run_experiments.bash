#!/bin/bash

# Directory containing ARGoS3 experiment files
EXPERIMENT_DIR="./experiments"
CONFIG_DIR="./agent_implementation/configs"
LOG_DIR="./logs"
ARGOSEXEC="argos3"

# Ensure log directory exists
mkdir -p "$LOG_DIR"

# List of experiment files (modify as needed)
EXPERIMENTS=("museum_config.argos")
CONFIGS=("config.yaml" "config2.yaml")


# Iterate over each experiment file
for EXPERIMENT in "${EXPERIMENTS[@]}"; do
    EXP_PATH="$EXPERIMENT_DIR/$EXPERIMENT"
#    export EXPERIMENT

    for CONFIG_FILE in "${CONFIGS[@]}"; do
        CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
#        export CONFIG_FILE

        METRIC_PATH="experiment_results/${EXPERIMENT%.argos}_${CONFIG_FILE%.yaml}"
        export METRIC_PATH
        FRAME_PATH="experiment_results/${EXPERIMENT%.argos}_${CONFIG_FILE%.yaml}/frames"

        sed "s|{{CONFIG_PATH}}|$CONFIG_PATH|g" "$EXP_PATH" > temp_experiment.argos
        sed "s|{{FRAME_PATH}}|${FRAME_PATH}|g" temp_experiment.argos > temp_experiment2.argos
        mkdir -p "$FRAME_PATH"
        # empty the directory
        rm -rf "$FRAME_PATH"/*

        # Check if the configuration file exists
        if [ ! -f "$EXP_PATH" ]; then
            echo "Error: Experiment config '$EXP_PATH' not found!" >&2
            continue
        fi

        # Generate timestamp for log file
        TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
        LOGFILE="$LOG_DIR/${EXPERIMENT%.argos}_${CONFIG_FILE%.yaml}_$TIMESTAMP.log"


        echo "Running ARGoS3 with configuration: $EXP_PATH and $CONFIG_PATH"

        # Run ARGoS3 and log output
        "$ARGOSEXEC" -c "temp_experiment2.argos" > "$LOGFILE" 2>&1

        # Check exit status
        if [ $? -eq 0 ]; then
            echo "Experiment '$EXPERIMENT' completed successfully."
        else
            echo "Experiment '$EXPERIMENT' failed. Check log: $LOGFILE" >&2
        fi
    done

done
rm temp_experiment.argos
rm temp_experiment2.argos

echo "All experiments finished."
