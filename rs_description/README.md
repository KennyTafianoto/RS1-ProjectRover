# HOW TO RUN
1. Build rs_description package
```
catkin_make --only-pkg-with-deps rs_description
```
2. Launch file
```
roslaunch rs_description basic-rocke.launch 
```

# CHANGING DIMENSIONS
Only change the xacro properties! 
```
<xacro:property name="chasis_width" value="0.6"/>
<xacro:property name="chasis_length" value="1"/>
<xacro:property name="chasis_height" value="0.2"/>
<xacro:property name="leg_radius" value="0.05"/>
<xacro:property name="leg_length" value="0.5"/>
<xacro:property name="leg_rotation" value="0.79"/>
```

# Resources
[URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
[Gazebo Plugins Tutorial](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Tutorial:UsingGazebopluginswithROS)
[Example 1: Four-wheeled-robot](https://github.com/harshmittal2210/Robotics_ws/tree/main/src/atom)
[ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros)
[Transmission](http://wiki.ros.org/urdf/XML/Transmission)

# Debugging Issues 
```
Issues with joint state publisher: 
rosdep install --from-paths src/ --ignore-src -r -y
```
