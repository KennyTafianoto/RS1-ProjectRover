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
    
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch




# April tags

## Installation:
    export ROS_DISTRO=noetic               # Set this to your distro, e.g. kinetic, melodic or noetic
    source /opt/ros/$ROS_DISTRO/setup.bash  # Source your ROS distro 
    mkdir -p ~/catkin_ws/src                # Make a new workspace 
    cd ~/catkin_ws/src                      # Navigate to the source space
    git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
    cd ~/catkin_ws                          # Navigate to the workspace
    rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
    catkin_make    # Build all packages in the workspace (catkin build and catkin_make_isolated will work also)


## SETUP:
### edit:
    src/apriltag_ros/apriltag_ros/launch/continuous_detection.launch
#### change parameters to match camera node names:
##### eg (usb_cam):
    <!-- configure camera input -->
      <arg name="camera_name" default="/usb_cam" />
      <arg name="image_topic" default="image_raw" />
      <arg name="queue_size" default="1" />
    
##### eg 2 (turtlebot3):
    <!-- configure camera input -->
    <arg name="camera_name" default="/camera" />
    <arg name="image_topic" default="/rgb/image_raw" />
    <arg name="queue_size" default="1" />

### Set tag family (using 36h11 by default):
#### edit:
    src/apriltag_ros/apriltag_ros/config/settings.yaml

### Add standalone tags (optional):
#### edit:
    src/apriltag_ros/apriltag_ros/config/tags.yaml
    standalone_tags:
#### add:
    standalone_tags: 
    [
        {id: 0, size: 1.0, description: "TAG_0"},
        {id: 1, size: 1.0, description: "TAG_1"},
        {id: 2, size: 1.0, description: "TAG_2"},
        {id: 3, size: 1.0, description: "TAG_3"},
        {id: 4, size: 1.0, description: "TAG_4"},
        {id: 5, size: 1.0, description: "TAG_5"}
    ]

    
### Calibrate Camera:
#### Install calibration library:
    rosdep install camera_calibration

#### Calibrate steps (change --square 0.024 to actual size in metres):
    roslaunch usb_cam usb_cam-test.launch
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/camera/image_raw camera:=/camera
Note: Save file to a location, the 'ost.yaml' file contains the useful data
    
#### Edit:
    src/launch/usb_cam-test.launch file

#### Add line (file location to match where you saved 'ost.yaml'):
    <param name="camera_info_url" value="file:///home/main/catkin_ws/src/usb_cam/cam_calibration/ost.yaml" />
    
### Launch a camera publishing node (options):
#### USB CAM:
    roslaunch usb_cam usb_cam-test.launch
#### turtleBot3:
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch rs_gazebo_world turtlebot3_marker.launch
    
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

### Launch detection:
    roslaunch apriltag_ros continuous_detection.launch
    rqt_image_view
        

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
