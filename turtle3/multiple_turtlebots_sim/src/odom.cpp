#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

void messageCb(const nav_msgs::Odometry::ConstPtr& cmd_po){
    ROS_INFO("position.x : %f\n", cmd_po->pose.pose.position.x);
    ROS_INFO("position.y : %f\n", cmd_po->pose.pose.position.y);
    ROS_INFO("position.z : %f\n", cmd_po->pose.pose.position.z);

    tf::Quaternion q(
    cmd_po->pose.pose.orientation.x,
    cmd_po->pose.pose.orientation.y,
    cmd_po->pose.pose.orientation.z,
    cmd_po->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("yaw : %f\n",yaw);
}
  
int main(int argc, char **argv){
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;
  
    // ros::Subscriber cmd_vel_subscriber(TOPIC_NAME, messageCb);
    ros::Subscriber cmd_po_subscriber =nh.subscribe("/robot2/odom", 10, messageCb);
    
    ros::spin();
  
    return 0;
}