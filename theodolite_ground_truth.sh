#!/bin/bash
screen -S "sensors" -X stuff $'\003'
screen -S "mapping" -X stuff $'\003'
screen -S "theodolite" -X stuff $'\003'
screen -S "icp_visualization" -X stuff $'\003'

echo "Starting the sensors"
screen -dmS sensors ros2 launch warthog_mapping sensors.launch.xml
echo "Sensors started, access it with screen -r sensors"

echo "Starting mapping"
screen -dmS mapping ros2 launch warthog_mapping realtime_mapping.launch.xml
echo "Mapping started, access it with screen -r mapping"
sleep 5

echo "Starting theodolite"
screen -dmS theodolite ros2 launch theodolite_pose theodolite_ground_truth.launch.py
echo "Theodolite started, access it with screen -r theodolite"

echo "Starting icp_visualization"
screen -dmS visualization ros2 launch warthog_mapping visualization.launch.xml
echo "Visualization started, access it with screen -r icp_visualization"

echo "Don't forget to save a rosbag using the command: ros2 launch warthog_mapping record.launch.xml"