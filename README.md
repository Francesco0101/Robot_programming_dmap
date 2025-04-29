# Robot_programming_dmap

This project implements a 2D localization system using ROS and a Distance Map. The robot receives an  occupancy map, an initial pose, and live laser scans. It estimates and continuously updates its corrected pose thanks to a laser scanner.

## Features

- Receives a map as an Occupancy Grid (`/map`)
- Receives an initial pose (`/initialpose`)
- Receives laser measurements (`/base_scan`)
- Uses a Distance Map to register laser scan endpoints and estimate the most likely current robot pose, then publishes the corrected odometry on `/localization/odom`

## Installation

### 1. Requirements
- ROS1 with Noetic on Ubuntu 20.04

### 2. Clone the repository

```bash
git clone https://github.com/Francesco0101/Robot_programming_dmap

```

### 3. Build the project

```bash

source opt/ros/noetic/setup.bash

catkin build

source devel/setup.bash
```
## Running the Node 
###   Each step should be run in a separate terminal in which you must run the commands:
``` bash
source opt/ros/noetic/setup.bash
source devel/setup.bash
```
### 1. Start ROS Core

```bash
roscore
```

### 2. Launch the node

```bash
rosrun dmap_localizer main_node

```

### 3. Publish the map

```bash
rosrun map_server map_server <path/to/map.yaml>

```

### 4. Start RViz for visualization
```bash
rviz

```

### 6. Start the similuation in which you can see the real position of the robot and move it around

```bash
rosrun stage_ros stageros <path/to/map.world>

```
