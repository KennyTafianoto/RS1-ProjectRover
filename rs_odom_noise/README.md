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
