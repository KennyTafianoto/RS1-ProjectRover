# To run:
### Build package:
```Ruby
cd ~/catkin_ws
catkin_make --pkg rs_odom_noise
source devel/setup.bash
```

### Roslaunch robot:
```
roslaunch blah blah blah
```

### For data logging:
Before rs_odom_noise_node starts, ensure that you are subscribing to a filter
topic that is already running (if you wish to log it).

### Start the node:

#### For just publishing noisy odometry:
```Ruby
rosrun rs_odom_noise rs_odom_noise_create_noise
```

#### For just recording data:
```Ruby
rosrun rs_odom_noise rs_odom_noise_record
```

#### For publishing noisy odometry and recording data at the same time:
```Ruby
rosrun rs_odom_noise rs_odom_noise_create_noise_and_record
```


# Using rosbag and saving CSV Data:
### Start roscore:
```Ruby
roscore
```

### Load rosbag (initially paused):
```Ruby
rosbag play --pause noisy_odometry.bag
```

### Enable filter/localisation (of your choice):
eg:
```Ruby
roslaunch robot_pose_ekf robot_pose_ekf.launch
```

### Start data recording node:
cd to your desired save location for the .csv file
```Ruby
rosrun rs_odom_noise rs_odom_noise_record
```

### Start rosbag:
press space in the window where 'rosbag play' entered

### Finish recording:
When rosbag is done, press `Ctrl+C` to stop recording to the .csv file
