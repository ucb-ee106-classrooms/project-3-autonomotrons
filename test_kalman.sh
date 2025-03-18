#!/bin/bash

# Define combinations of Q, P, and R
combinations=(
  "1.0 1.0 1.0"
  "2.0 2.0 0.5"
  "3.0 1.5 0.8"
)

# Python script to run
PYTHON_SCRIPT="analysis.py"
OUTPUT_FILE="output.log"

# Clear previous output file
echo "Starting new tests. Results will be saved to $OUTPUT_FILE"
echo "" > $OUTPUT_FILE

# Loop through each combination
for combo in "${combinations[@]}"; do
  read -r Q P R <<< "$combo"
  echo "Running with Q=$Q, P=$P, R=$R" | tee -a $OUTPUT_FILE

  # Launch ROS with specified parameters
  gnome-terminal -- bash -c "roslaunch proj3_pkg estimator.launch estimator_type:=kalman_filter Qmult:=$Q Pmult:=$P Rmult:=$R; exec bash" &
  LAUNCH_PID=$!

  # Wait for 15 seconds
  echo "Waiting for 15 seconds..." | tee -a $OUTPUT_FILE
  sleep 15

  # Send Ctrl+C to the launch process
  echo "Stopping ROS launch..." | tee -a $OUTPUT_FILE
  kill -SIGINT $LAUNCH_PID
  wait $LAUNCH_PID

  # Run the Python script and save its output
  echo "Running Python script: $PYTHON_SCRIPT" | tee -a $OUTPUT_FILE
  gnome-terminal -- bash -c "python3 $PYTHON_SCRIPT | tee -a $OUTPUT_FILE; exec bash" &
  PYTHON_PID=$!

  # Wait for 1 second
  echo "Waiting for 1 second..." | tee -a $OUTPUT_FILE
  sleep 1

  # Send Ctrl+C to the Python script
  echo "Stopping Python script..." | tee -a $OUTPUT_FILE
  kill -SIGINT $PYTHON_PID
  wait $PYTHON_PID

  echo "Test with Q=$Q, P=$P, R=$R completed." | tee -a $OUTPUT_FILE
  echo "----------------------------------------------" | tee -a $OUTPUT_FILE
done

echo "All tests completed. Results are saved in $OUTPUT_FILE."
