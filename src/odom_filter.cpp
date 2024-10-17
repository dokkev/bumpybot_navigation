#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomFilter{
public:
    OdomFilter()
    : nh_(),
      odom_pub_(nh_.advertise<nav_msgs::Odometry>("odom_icp_filtered", 10)),
      odom_sub_(nh_.subscribe("odom_icp", 10, &OdomFilter::odomCallback, this))
    {
        prev_odom_ = std::make_shared<nav_msgs::Odometry>();
        filtered_odom_ = std::make_shared<nav_msgs::Odometry>();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Apply low-pass filter
        double alpha = 0.1;  // Filter coefficient (between 0 and 1)

        filtered_odom_->pose.pose.position.x = alpha * msg->pose.pose.position.x + (1 - alpha) * prev_odom_->pose.pose.position.x;
        filtered_odom_->pose.pose.position.y = alpha * msg->pose.pose.position.y + (1 - alpha) * prev_odom_->pose.pose.position.y;
        filtered_odom_->pose.pose.position.z = alpha * msg->pose.pose.position.z + (1 - alpha) * prev_odom_->pose.pose.position.z;

        // Compute orientation quaternion
        filtered_odom_->pose.pose.orientation.x = msg->pose.pose.orientation.x;
        filtered_odom_->pose.pose.orientation.y = msg->pose.pose.orientation.y;
        filtered_odom_->pose.pose.orientation.z = msg->pose.pose.orientation.z;
        filtered_odom_->pose.pose.orientation.w = msg->pose.pose.orientation.w;

        // Store current odometry for next iteration
        *prev_odom_ = *filtered_odom_;

        // Publish filtered odometry
        odom_pub_.publish(*filtered_odom_);

        // Send TF transform
        sendOdomTf(*filtered_odom_);
    }

    void sendOdomTf(const nav_msgs::Odometry& msg)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = msg.pose.pose.position.x;
        odom_trans.transform.translation.y = msg.pose.pose.position.y;
        odom_trans.transform.translation.z = msg.pose.pose.position.z;

        geometry_msgs::Quaternion odom_quat;

        odom_quat.x = msg.pose.pose.orientation.x;
        odom_quat.y = msg.pose.pose.orientation.y;
        odom_quat.z = msg.pose.pose.orientation.z;
        odom_quat.w = msg.pose.pose.orientation.w;

        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster_.sendTransform(odom_trans);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster odom_broadcaster_;
    std::shared_ptr<nav_msgs::Odometry> prev_odom_;
    std::shared_ptr<nav_msgs::Odometry> filtered_odom_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry_filter");
    
    OdomFilter odom_filter;

    ros::spin();
    
    return 0;
}




