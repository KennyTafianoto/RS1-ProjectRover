#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np

# Global variables for position and velocity
position = Vector3(0, 0, 0)
velocity = Vector3(0, 0, 0)
last_time = None

# Callback function to process IMU data
def imu_callback(msg):
    # rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher('/odom_localisation', Odometry, queue_size = 10)
    
    global last_time
    global position
    global velocity

    if last_time is None:
        last_time = msg.header.stamp
        return

    # Calculate the time difference
    dt = (msg.header.stamp - last_time).to_sec()

    # Integrate linear accelerations to estimate velocity
    velocity.x += msg.linear_acceleration.x * dt
    velocity.y += msg.linear_acceleration.y * dt
    velocity.z += msg.linear_acceleration.z * dt

    # Integrate velocity to estimate position
    position.x += velocity.x * dt
    position.y += velocity.y * dt
    position.z += velocity.z * dt

    # Update the last time
    last_time = msg.header.stamp

    # Print the current position
    rospy.loginfo("Current Position (x, y, z): (%.2f, %.2f, %.2f)", position.x, position.y, position.z)

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom_l"
    odom.pose.pose = Pose(Point(position.x, position.y, position.z), msg.orientation)

    odom_pub.publish(odom)

    

def imu_listener():
    # Initialize the ROS node
    rospy.init_node('imu_localization', anonymous=True)

    # Subscribe to the /imu topic with the Imu message type
    rospy.Subscriber("/imu", Imu, imu_callback)

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass
