# Using Particle filter / Monte Carlo Localisation

## For TurtleBot3

### Grid Mapping:

### Edit parameters:
```Ruby
gedit ~/catkin_ws/src/turtlebot3/turtlebot3_slam/config/gmapping_params.yaml
```
#### Replace with:
```Ruby
map_update_interval: 2.0
maxUrange: 3.0
sigma: 0.05
kernelSize: 1
lstep: 0.05
astep: 0.05
iterations: 5
lsigma: 0.075
ogain: 3.0
lskip: 0
minimumScore: 50
srr: 0.1
srt: 0.2
str: 0.1
stt: 0.2
linearUpdate: 1.0
angularUpdate: 0.2
temporalUpdate: 0.5
resampleThreshold: 0.5
particles: 100
xmin: -10.0
ymin: -10.0
xmax: 10.0
ymax: 10.0
delta: 0.05
llsamplerange: 0.01
llsamplestep: 0.01
lasamplerange: 0.005
lasamplestep: 0.005
```

```Ruby
export TURTLEBOT3_MODEL=waffle
roslaunch rs_gazebo_world turtlebot3_marker_V2.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Using Particle Filter:
```Ruby
export TURTLEBOT3_MODEL=waffle
roslaunch rs_gazebo_world turtlebot3_marker_V2.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_name.yaml
```










# installing kalman filter

### Clone robot_pose_ekf library:
```ruby
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/robot_pose_ekf.git

cd ~/catkin_Ws
catkin_make
source devel/setup.bash
```

### Edit launch file configuration:
```ruby
gedit ~/catkin_ws/src/robot_pose_ekf/robot_pose_ekf.launch
```

#### Replace with:
```
<launch>

<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
  <param name="output_frame" value="odom_combined"/>
  <param name="base_footprint_frame" value="base_footprint_ekf"/>
  <param name="freq" value="30.0"/>
  <param name="sensor_timeout" value="1.0"/>  
  <param name="odom_used" value="true"/>
  <param name="imu_used" value="true"/>
  <param name="vo_used" value="false"/>

  <remap from="imu" to="/imu" />
</node>

</launch>
```

### Test:
```ruby
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

roslaunch robot_pose_ekf robot_pose_ekf.launch

rostopic echo /robot_pose_ekf/odom_combined
```
