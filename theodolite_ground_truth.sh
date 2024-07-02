#!/bin/bash
screen -S "theodolite" -X quit
screen -S "icp_visualization" -X quit

echo "Starting theodolite"
screen -dmS theodolite ros2 launch theodolite_pose theodolite_ground_truth.launch.py
echo "Theodolite started, access it with screen -r theodolite"

echo "Starting icp_visualization"
screen -dmS visualization ros2 launch warthog_mapping visualization.launch.xml
echo "Visualization started, access it with screen -r icp_visualization"

echo "Don't forget to save a rosbag"
