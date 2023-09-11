#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TagDetectionNode
{
public:
    TagDetectionNode(ros::NodeHandle& nh)
    {
        nh_ = nh;

        tag_detection_sub_ = nh_.subscribe("tag_detections", 10, &TagDetectionNode::detectedTagCallback, this);
        imu_sub_ = nh_.subscribe("/imu", 10, &TagDetectionNode::imuCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/rgbd_odom", 10);
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_detection_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;
    sensor_msgs::Imu global_imu_;

    geometry_msgs::Pose convertToBaseLinkFrame(const geometry_msgs::Pose& input_pose, const std::string& frame_id)
    {
        tf::TransformListener listener;
        tf::StampedTransform transform;

        try
        {
            listener.waitForTransform("/base_link", frame_id, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/base_link", frame_id, ros::Time(0), transform);

            tf::Pose input_tf_pose;
            tf::poseMsgToTF(input_pose, input_tf_pose);
            tf::Pose base_link_tf_pose = transform * input_tf_pose;

            geometry_msgs::Pose base_link_pose;
            tf::poseTFToMsg(base_link_tf_pose, base_link_pose);

            return base_link_pose;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Transform error: %s", ex.what());
            return input_pose;
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
    {
        global_imu_ = *imu;
    }

    geometry_msgs::Pose rotatePoint(double yaw, geometry_msgs::Pose pose) 
    {
        geometry_msgs::Pose out_pose;

        // Extract the input point coordinates
        double x = pose.position.x;
        double y = pose.position.y;

        // Calculate the rotated coordinates
        double cos_theta = cos(yaw);
        double sin_theta = sin(yaw);

        out_pose.position.x = x * cos_theta - y * sin_theta;
        out_pose.position.y = x * sin_theta + y * cos_theta;
        out_pose.position.z = pose.position.z;

        return out_pose;
    }

    double quaternionToYaw(const geometry_msgs::Quaternion& quat)
    {
        tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 mat(tf_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        return yaw;
    }

    void detectedTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& detection_msg)
    {
        nav_msgs::Odometry odom_msg;
        geometry_msgs::Point tag0, tag1, tag3, tag4, tag5, global;
        geometry_msgs::Pose odom_;
        tf::pointTFToMsg(tf::Vector3(-1.955090, 3.283930, 0.505621), tag0);
        tf::pointTFToMsg(tf::Vector3(3.618770, 3.480450, 0.480635), tag1);
        tf::pointTFToMsg(tf::Vector3(-1.788420, 6.437140, 0.491476), tag3);
        tf::pointTFToMsg(tf::Vector3(3.294030, 3.533260, 0.466723), tag4);
        tf::pointTFToMsg(tf::Vector3(-1.380480, 13.830400, 0.502502), tag5);

        for (auto it = detection_msg->detections.begin(); it != detection_msg->detections.end(); ++it)
        {
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

            geometry_msgs::Pose conv_pose;
            conv_pose = convertToBaseLinkFrame((*it).pose.pose.pose, (*it).pose.header.frame_id);
            conv_pose = rotatePoint(quaternionToYaw(global_imu_.orientation), conv_pose);

            odom_.position.x += global.x - conv_pose.position.x;
            odom_.position.y += global.y - conv_pose.position.y;
            odom_.position.z += global.z - conv_pose.position.z;
        }

        odom_msg.pose.pose.position.x = odom_.position.x / detection_msg->detections.size();
        odom_msg.pose.pose.position.y = odom_.position.y / detection_msg->detections.size();
        odom_msg.pose.pose.position.z = odom_.position.z / detection_msg->detections.size();

        ROS_INFO_STREAM("avg X: " + std::to_string(odom_msg.pose.pose.position.x) + " Y: " + std::to_string(odom_msg.pose.pose.position.y) + " Z: " + std::to_string(odom_msg.pose.pose.position.z));

        odom_pub_.publish(odom_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_detection_node");
    ros::NodeHandle nh;

    TagDetectionNode node(nh);
    node.run();

    return 0;
}