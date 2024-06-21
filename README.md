# theodolite_pose
A package for recording ground truth poses of a theodolite with the Warthog robot.

## Installation
Clone the repository in your ROS2 workspace and build it.

```
git clone git@github.com:norlab-ulaval/theodolite_pose.git
```

## Usage
### Launch the theodolite_pose node
```
ros2 launch theodolite_pose theodolite_pose.launch.py
```
Subscribe to the ```/theodolite_master``` and publishes the topic ```/theodolite_pose``` containing the ground truth pose in the ```theodolite``` frame.
Then compute the calibration aiming ```prism1```, ```prism2``` then ```prism3``` on the same ```channel 1``` and going back to the ```prism1``` to publish the theodolite to map transformation (cf Transforms).
### Launch icp_theodolite_pose node
```
ros2 launch theodolite_pose icp_theodolite_pose.launch.py
```
Subscribe to the ```/icp_odom``` and publishes the topic ```/icp_theodolite_pose``` containing the ```icp odometry``` in ```map``` frame at ```prism1``` position instead of ```base_link```. 

## Transforms
![image](https://github.com/norlab-ulaval/theodolite_pose/assets/123117664/f8bc4dbc-4fe3-4c76-951b-9518dabf62ee)
