# GPS Localization and Object Detection Package

## Overview
This ROS2 package combines GPS-based navigation with object detection and identification, enhancing robots' capabilities for environmental interaction. Inspired by [Pedro Gonzalez's tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html) on using GPS with Nav2 and robot_localization, it extends functionality by integrating a vision node for object detection and distance measurement.

## Setup Instructions
### Prerequisites
- ROS2 Humble
- Nav2
- Robot_localization package
- Ultralytics YOLOv8

### Installation
Clone this package into your ROS2 workspace and build it with colcon:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build --symlink-install
```
## Running the Package
To launch the GPS waypoint follower with RViz for interactive goal setting, use the following command:

```bash
ros2 launch gps_nav2 gps_waypoint_follower.launch.py use_rviz:=True
```
To enable interactive GPS waypoint setting through Mapviz, this package includes the `interactive_waypoint_follower` node. This feature allows users to click on a location within Mapviz to set it as a navigation goal for the robot. The node handles the conversion of geographic coordinates (latitude and longitude) to ROS poses, which are then used by Nav2 for navigation.

To use this feature, launch the package with Mapviz and start the `interactive_waypoint_follower`:

```bash
ros2 launch gps_nav2 gps_waypoint_follower.launch.py use_mapviz:=True
ros2 run gps_nav2 interactive_waypoint_follower
```

The vision node in this package utilizes the YOLOv8 model to detect objects within the robot's environment. You can visualize these detections by subscribing to the /visualized_detections topic in RViz. The visualized image will display bounding boxes around detected objects, along with their class, confidence, and distance from the robot.

To start the vision node, use the following command:

```bash
ros2 run gps_nav2 vision_node
```