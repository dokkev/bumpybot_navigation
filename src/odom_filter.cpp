#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


// Global variables
std::shared_ptr<nav_msgs::Odometry> prev_odom(new nav_msgs::Odometry);
std::shared_ptr<nav_msgs::Odometry> filtered_odom(new nav_msgs::Odometry);
tf::TransformBroadcaster odom_broadcaster;
ros::Publisher odom_pub;

void sendOdomTf(const nav_msgs::Odometry& msg, tf::TransformBroadcaster& odom_broadcaster);

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Apply low-pass filter
    double alpha = 0.1;  // Filter coefficient (between 0 and 1)

    filtered_odom->pose.pose.position.x = alpha * prev_odom->pose.pose.position.x + (1 - alpha) * msg->pose.pose.position.x;
    filtered_odom->pose.pose.position.y = alpha * prev_odom->pose.pose.position.y + (1 - alpha) * msg->pose.pose.position.y;
    filtered_odom->pose.pose.position.z = alpha * prev_odom->pose.pose.position.z + (1 - alpha) * msg->pose.pose.position.z;

    // Store current odometry for next iteration
    *prev_odom = *filtered_odom;

    // Publish filtered odometry
    odom_pub.publish(*filtered_odom);

    // Broadcast the transform
    sendOdomTf(*filtered_odom, odom_broadcaster);
 

}

void sendOdomTf(const nav_msgs::Odometry& msg, tf::TransformBroadcaster& odom_broadcaster) {
    // Create the transformation
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg.header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // Copy the odometry information
    odom_trans.transform.translation.x = msg.pose.pose.position.x;
    odom_trans.transform.translation.y = msg.pose.pose.position.y;
    odom_trans.transform.translation.z = msg.pose.pose.position.z;
    odom_trans.transform.rotation = msg.pose.pose.orientation;

    // Broadcast the transform
    odom_broadcaster.sendTransform(odom_trans);
}

    


int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_filter");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom_icp_filtered", 10);
    

    ros::spin();

    return 0;
}