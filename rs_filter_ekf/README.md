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
