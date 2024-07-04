#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/robot/ros2_ws/install/local_setup.bash
source /home/robot/ros2_ws/src/norlab_robot/scripts/bash/env.sh

screen -r -S "sensors" -X quit
screen -r -S "mapping" -X quit
screen -r -S "theodolite" -X quit
screen -r -S "icp_visualization" -X quit
screen -r -S "record" -X quit

echo "Starting sensors..."
screen -dmS sensors ros2 launch norlab_robot sensors.launch.py
echo "Sensors started, access it with screen -r sensors"
echo "-----------------------------"

echo "Starting mapping..."
screen -dmS mapping ros2 launch norlab_robot mapping.launch.py
echo "Mapping started, access it with screen -r mapping"
echo "-----------------------------"

echo "Starting theodolite"
screen -dmS theodolite ros2 launch theodolite_pose theodolite_pose.launch.py
echo "Theodolite started, access it with screen -r theodolite"
echo "-----------------------------"

echo "Starting icp_visualization"
screen -dmS visualization ros2 launch theodolite_pose icp_pose.launch.py
echo "Visualization started, access it with screen -r icp_visualization"
echo "-----------------------------"

echo "Starting recording"
screen -dmS record ros2 launch norlab_robot rosbag_record.launch.py
echo "Record started, acces it with screen -r record"
echo "-----------------------------"

