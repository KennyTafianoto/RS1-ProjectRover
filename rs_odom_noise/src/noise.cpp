#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <random>
#include <fstream>
#include <iostream>
#include <cstdlib>

ros::Publisher noisy_odom_publisher;

std::ofstream csv_file; 

double getRandomNoise() {
    // Generate random noise between -5% and 5%
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-0.05, 0.05);
    return dis(gen);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    // Create a new Odometry message for the noisy data
    nav_msgs::Odometry noisy_odom;

    noisy_odom.pose.pose.position.x = odom_msg->pose.pose.position.x * (1 + getRandomNoise());
    noisy_odom.pose.pose.position.y = odom_msg->pose.pose.position.y * (1 + getRandomNoise());
    noisy_odom.pose.pose.position.z = odom_msg->pose.pose.position.z * (1 + getRandomNoise());

    // Publish the noisy odometry data to a different topic
    noisy_odom_publisher.publish(noisy_odom);

    csv_file << odom_msg->pose.pose.position.x << "," << noisy_odom.pose.pose.position.x << ","
             << odom_msg->pose.pose.position.y << "," << noisy_odom.pose.pose.position.y << ","
             << odom_msg->pose.pose.position.z << "," << noisy_odom.pose.pose.position.z << "\n";
    csv_file.flush(); 

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "noisy_odometry_publisher");
    ros::NodeHandle nh;

    std::string logFilePath = ros::package::getPath("RS1-ProjectRover/rs_odom_noise") + "/odometry_data.csv";
    // std::string logFilePath = "/odometry_data.csv";

    csv_file.open(logFilePath);
    if (!csv_file.is_open()) {
        ROS_ERROR("Failed to open the CSV file for writing.");
        return 1;
    }

    csv_file << "Original_X,Noisy_X,Original_Y,Noisy_Y,Original_Z,Noisy_Z\n";

    // Subscriber to the odometry topic
    ros::Subscriber odom_subscriber = nh.subscribe("/odom", 10, odomCallback);

    // Publisher for the noisy odometry
    noisy_odom_publisher = nh.advertise<nav_msgs::Odometry>("/noisy_odom", 10);

    ros::spin();
    return 0;
}
