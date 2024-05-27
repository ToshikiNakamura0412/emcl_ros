# emcl_ros

![Build Status](https://github.com/ToshikiNakamura0412/emcl_ros/workflows/build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS implementation of emcl (mcl with expansion resetting)

<p align="center">
  <img src="https://github.com/ToshikiNakamura0412/amr_navigation_gifs/blob/master/images/emcl_demo1.gif" height="240px"/>
  <img src="https://github.com/ToshikiNakamura0412/amr_navigation_gifs/blob/master/images/emcl_demo2.gif" height="240px"/>
  <img src="https://github.com/ToshikiNakamura0412/amr_navigation_gifs/blob/master/images/emcl_demo3.gif" height="240px"/>
  <img src="https://github.com/ToshikiNakamura0412/amr_navigation_gifs/blob/master/images/emcl_demo4.gif" height="240px"/>
</p>

## Environment
- Ubuntu 20.04
- ROS Noetic

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/emcl_ros.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic # Install dependencies
catkin build emcl_ros -DCMAKE_BUILD_TYPE=Release        # Release build is recommended
```

## How to use
```
roslaunch emcl_ros emcl_ros.launch
```

## Running the demo
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/scan_to_pcl_ros.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic
catkin build -DCMAKE_BUILD_TYPE=Release

# run demo
## terminal 1
export TURTLEBOT3_MODEL=burger
roslaunch emcl_ros test.launch
## terminal 2
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## Node I/O
![Node I/O](images/node_io.png)

## Nodes
### emcl
#### Published Topics
- /emcl_pose (`geometry_msgs/PoseWithCovarianceStamped`)
  - ~~~
- /tf (`tf2_msgs/TFMessage`)
  - ~~~
- ~\<name>/particle_cloud (`geometry_msgs/PoseArray`)
  - ~~~

#### Subscribed Topics
- /cloud (`sensor_msgs/PointCloud2`)
  - ~~~
- /initialpose (`geometry_msgs/PoseWithCovarianceStamped`)
  - ~~~
- /odom (`nav_msgs/Odometry`)
  - ~~~
- /scan (`sensor_msgs/LaserScan`)
  - ~~~


#### Parameters
##### EMCL Parameters
- ~\<name>/<b>expansion_position_dev</b> (float: `0.07` [m]):<br>
- ~\<name>/<b>expansion_orientation_dev</b> (float, default: `0.15` [rad]):<br>
- ~\<name>/<b>init_x</b> (float, default: `0` [m]):<br>
- ~\<name>/<b>init_y</b> (float, default: `0` [m]):<br>
- ~\<name>/<b>init_yaw</b> (float, default: `0` [rad]):<br>
- ~\<name>/<b>init_position_dev</b> (float, default: `0.1` [m]):<br>
- ~\<name>/<b>init_orientation_dev</b> (float, default: `0.05` [rad]):<br>
- ~\<name>/<b>laser_step</b> (int, default: `4`):<br>
- ~\<name>/<b>likelihood_th</b> (float, default: `0.002`):<br>
- ~\<name>/<b>particle_num</b> (int, default: `420`):<br>
- ~\<name>/<b>reset_count_limit</b> (int, default: `3`):<br>
- ~\<name>/<b>sensor_noise_ratio</b> (float, default: `0.02`):<br>
- ~\<name>/<b>use_cloud</b> (bool, default: `False`):<br>

##### OdomModel Parameters
- ~\<name>/<b>ff</b> (float, default: `0.17` [m]):<br>
- ~\<name>/<b>fr</b> (float, default: `0.0005` [m]):<br>
- ~\<name>/<b>rf</b> (float, default: `0.13` [rad]):<br>
- ~\<name>/<b>rr</b> (float, default: `0.2` [rad]):<br>

##### Scan Parameters
- ~\<name>/<b>range_min</b> (float, default: `0.12` [m]):<br>
- ~\<name>/<b>range_max</b> (float, default: `3.5` [m]):<br>

## References
- url
