#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

ros::Publisher odom_pub;

void detectedtagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& detection_msg)
{
    // Process tag detections and create an Odometry message
    nav_msgs::Odometry odom_msg;

    // Establish ground truth (Global coordinates of april tags)
    geometry_msgs::Point tag0, tag1, tag3, tag4, tag5, global, odom;
    tf::pointTFToMsg(tf::Vector3(-1.955090, 3.283930, 0.505621), tag0);
    tf::pointTFToMsg(tf::Vector3(3.618770, 3.480450, 0.480635), tag1);
    tf::pointTFToMsg(tf::Vector3(-1.788420, 6.437140, 0.491476), tag3);
    tf::pointTFToMsg(tf::Vector3(4.349050, 9.093160, 0.465172), tag4);
    tf::pointTFToMsg(tf::Vector3(-1.380480, 13.830400, 0.502502), tag5);

    for(auto it = detection_msg->detections.begin(); it != detection_msg->detections.end(); ++it)
    {
        // Calculate Global coordinates of camera from april tag
        switch ((*it).id.at(0))
        {
        case 0:
            global = tag0;
            break;
        case 1:
            global = tag1;
            break;
        case 3:
            global = tag3;
            break;
        case 4:
            global = tag4;
            break;
        case 5:
            global = tag5;
            break;
        }

        // ROS_INFO_STREAM("ID: " + std::to_string((*it).id.at(0)));
        // ROS_INFO_STREAM("Test");

        odom.x += global.x - (*it).pose.pose.pose.position.x;
        odom.y += global.y - (*it).pose.pose.pose.position.y;
        odom.z += global.z - (*it).pose.pose.pose.position.z;

    // ROS_INFO_STREAM("detected X: " + std::to_string(global.x - (*it).pose.pose.pose.position.x)+ " Y: " + std::to_string(global.y - (*it).pose.pose.pose.position.y) + " Z: " + std::to_string(global.z - (*it).pose.pose.pose.position.z));
    ROS_INFO_STREAM("detected X: " + std::to_string((*it).pose.pose.pose.position.x)+ " Y: " + std::to_string((*it).pose.pose.pose.position.y) + " Z: " + std::to_string((*it).pose.pose.pose.position.z));
    ROS_INFO_STREAM("global X: " + std::to_string(global.x)+ " Y: " + std::to_string(global.y) + " Z: " + std::to_string(global.z));
    }

    odom_msg.pose.pose.position.x = odom.x/detection_msg->detections.size();
    odom_msg.pose.pose.position.y = odom.y/detection_msg->detections.size();
    odom_msg.pose.pose.position.z = odom.z/detection_msg->detections.size();

    ROS_INFO_STREAM("avg X: " + std::to_string(odom_msg.pose.pose.position.x)+ " Y: " + std::to_string(odom_msg.pose.pose.position.y) + " Z: " + std::to_string(odom_msg.pose.pose.position.z));
    
    // Publish the Odometry message
    odom_pub.publish(odom_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber tag_detection_sub = nh.subscribe("tag_detections", 10, detectedtagCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/rgbd_odom", 10);

    ros::spin();

    return 0;
}