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
### edit:
    src/apriltag_ros/apriltag_ros/launch/continuous_detection.launch
### change (values to match camera node names):
    eg (usb_cam):
        <!-- configure camera input -->
          <arg name="camera_name" default="/usb_cam" />
          <arg name="image_topic" default="image_raw" />
          <arg name="queue_size" default="1" />
    
    eg 2 (turtlebot3):
         <!-- configure camera input -->
      <arg name="camera_name" default="/camera" />
      <arg name="image_topic" default="/rgb/image_raw" />
      <arg name="queue_size" default="1" />

### Set tag family:
#### edit:
        src/apriltag_ros/apriltag_ros/config/settings.yaml

### Add standalone tags (optional):
#### edit:
        src/apriltag_ros/apriltag_ros/config/tags.yaml

    
### Calibrate Camera:
#### Install calibration library:
        rosdep install camera_calibration

#### Calibrate:
        roslaunch usb_cam usb_cam-test.launch
        rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/camera/image_raw camera:=/camera
        save file somewhere
    
##$# Edit:
    usb_cam-test.launch file

#### Add line:
        <param name="camera_info_url" value="file:///home/main/catkin_ws/src/usb_cam/cam_calibration/ost.yaml" />
    

### Launch:
    roslaunch usb_cam usb_cam-test.launch
    roslaunch apriltag_ros continuous_detection.launch
    rqt_image_view
        
