# RS1-ProjectRover
Rover Project for Robotics Studio 1
## Naming structure
rs_<packagename>
e.g. rs_gazebo
     rs_description

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
