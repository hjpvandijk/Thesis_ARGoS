#!/bin/bash

# Directory containing ARGoS3 experiment files
EXPERIMENT_DIR="./experiments"
LOG_DIR="./logs"
ARGOSEXEC="argos3"

# Ensure log directory exists
mkdir -p "$LOG_DIR"

# List of experiment files (modify as needed)
EXPERIMENTS=("hugo_experiment.argos")

# Iterate over each experiment file
for EXPERIMENT in "${EXPERIMENTS[@]}"; do
    CONFIG_PATH="$EXPERIMENT_DIR/$EXPERIMENT"
    export EXPERIMENT

    # Check if the configuration file exists
    if [ ! -f "$CONFIG_PATH" ]; then
        echo "Error: Experiment config '$CONFIG_PATH' not found!" >&2
        continue
    fi

    # Generate timestamp for log file
    TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
    LOGFILE="$LOG_DIR/${EXPERIMENT%.argos}_$TIMESTAMP.log"

    echo "Running ARGoS3 with configuration: $CONFIG_PATH"

    # Run ARGoS3 and log output
    "$ARGOSEXEC" -c "$CONFIG_PATH" > "$LOGFILE" 2>&1

    # Check exit status
    if [ $? -eq 0 ]; then
        echo "Experiment '$EXPERIMENT' completed successfully."
    else
        echo "Experiment '$EXPERIMENT' failed. Check log: $LOGFILE" >&2
    fi
done

echo "All experiments finished."
