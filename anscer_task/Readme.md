# ROS Trajectory Management System

This ROS package consists of two nodes for managing trajectory data in a ROS environment. The first node collects trajectory data from an odometry topic and publishes visualization markers representing the trajectory. It also provides a service to save the trajectory data to a CSV file. User can specifiy time duration for what latest time they want it to be present

The second node reads trajectory data from a specified CSV file given by the user and transforms the trajectory data to the odom frame and publishes for visualization representing the trajectory in RViz.

## Nodes

### 1. Trajectory Publisher Node

This node subscribes to the odometry topic (/odom) and collects trajectory data. It then publishes visualization markers representing the trajectory to RViz. Additionally, it provides a service to save the trajectory data to a file.

#### Subscribed Topics
- `/odom` (nav_msgs/Odometry): Odometry data containing the trajectory information.

#### Published Topics
- `/trajectory_markers` (visualization_msgs/MarkerArray): Visualization markers representing the trajectory.

#### Services
- `saveTrajectory` (anscer_task/saveTrajectory): Service to save the trajectory data to a file.

### 2. Trajectory Reader Node

This node reads trajectory data from a CSV file and publishes visualization markers representing the trajectory to RViz.

#### Published Topics
- `/visualisation_marker_array` (visualization_msgs/MarkerArray): Visualization markers representing the trajectory.



## Usage

### Starting SLAM

Before starting it, we need to start SLAM so that it can autonomously move and publish the odom data, Here Anscer's Packagae AR100 is used for the same purpose. So before starting this, run the simulation, create a map and test out if the bot is working perfectly.

### Trajectory Publisher Node

Run the trajectory publisher node with the following command:

```bash
rosrun anscer_task anscer_task 

```
### Saving Trajectory Data

To save the trajectory data using the service provided by the trajectory publisher node, you can use rosservice command:
```bash
rosservice call /saveTrajectory "<filename: 'path/to/save/file.csv'> <duration: 50.0>"
```
### Trajectory Publisher Node

Run the trajectory publisher node with the following command:

```bash
rosrun anscer_task anscer_task_node_2<path/to/trajectory_file>
```
