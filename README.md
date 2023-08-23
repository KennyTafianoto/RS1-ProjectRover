# RS1-ProjectRover
Rover Project for Robotics Studio 1

## Cloning the Repository
    git clone git@github.com:KennyTafianoto/RS1-ProjectRover.git
    cd ~/catkin_ws/src/RS1-ProjectRover/rs_gazebo_world/model
    cp -R .* ~/.gazebo/models/
    cd ~/catkin_ws
    catkin_make

## Launching the Simulation
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch rs_gazebo_world turtlebot3_marker.launch
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

### Reference List
April Tag Models: https://github.com/koide3/gazebo_apriltag.git  
AR Tracking 1: http://wiki.ros.org/apriltag_ros  
AR Tracking 2: http://wiki.ros.org/ar_track_alvar  
Turtlebot3 + D435i:
- https://www.youtube.com/watch?v=hpUCG6K5muI
- https://github.com/rickstaa/realsense-ros-gazebo
- https://github.com/pal-robotics-forks/realsense
- https://github.com/pal-robotics-forks/realsense/tree/kinetic-devel/realsense2_description/urdf

SLAM with D435i:
- https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i


## April tags
    edit src/apriltag_ros/apriltag_ros/launch/continuous_detection.launch
    change "configure camera input" values to match camera node
    
    eg:
        <!-- configure camera input -->
          <arg name="camera_name" default="/usb_cam" />
          <arg name="image_topic" default="image_raw" />
          <arg name="queue_size" default="1" />
