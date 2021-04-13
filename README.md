# localization_system ROS package
This package is used to estimate a car's position in UTM coordinate (x,y) based on data from GNSS, IMU, and wheel speed sensor that is fused using Unscented Kalman Filter.

## Requirements
1. [Robot Operating System (ROS)](https://www.ros.org/), this package is tested on [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
2. [gps_common](https://github.com/swri-robotics/gps_umd) ROS package
3. [rviz_satellite](https://github.com/nobleo/rviz_satellite) ROS package (optional)
4. [mapviz](https://github.com/swri-robotics/mapviz) ROS package (optional)

## Installation
1. Clone this package and add it to your catkin workspace.
2. Run `catkin_make` or `catkin build`, depending on your workspace.

## Usage
For general purposes, run this command on your terminal:
```bash
roslaunch localization_system ukf_localization_2d.launch
```
For a quick demo, run this command on your terminal:
```bash
roslaunch localization_system demo_ukf_localization.launch
```
### Important Notes
1. This package expects a topic called `/imu` for imu message and a topic called `/sensor_velocity` for wheel speed sensor message. You can change the topic names in the nodes located in src.
2. For GNSS message, you can set it in the gps_umd package in gps_common/src/utm_odometry_node.cpp. To be able to use this package, you have to change the published topic to `/utm` and set the subscribed topic depending on your GNSS topic.
3. If you want to activate a satellite-map in the Rviz, check the AerialMapDisplay checkbox in the Rviz config settings.

## Preview
This is an example of when you run the demo and activate the AerialMapDisplay.
![demo preview](https://github.com/anzulfa/localization_system/blob/main/demo/demo.png?raw=true)

## Additional
### Relative Localization
This package also contains `relative_localization_2d` node for estimating the car's position using relative localization method that only uses data from wheel speed sensor and IMU. To use this node, run:
```bash
roslaunch localization_system relative_localization_2d.launch
```
For a quick relative localization demo, run:
```bash
roslaunch localization_system demo_relative_localization.launch
```

### Mapviz
You can also view the 2D visualization of the localization system using [mapviz](https://github.com/swri-robotics/mapviz). This visualization uses google map plugin for mapviz created by Daniel Snider. You can find the complete documentation [here](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite).

In order to show the satellite map in mapviz, you have to install [docker](https://docs.docker.com/engine/install/) in your system, then run this commands:
```bash
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```
To visualize UKF localization on mapviz, run:
```bash
roslaunch localization_system mapviz_ukf_localization_2d.launch
```
To visualize relative localization on mapviz, run:
```bash
roslaunch localization_system mapviz_relative_localization_2d.launch
```

Mapviz visualization preview:
![Mapviz preview](https://github.com/anzulfa/localization_system/blob/main/demo/mapviz_example.png?raw=true)