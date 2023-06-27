#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
  
int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle nh;
  geometry_msgs::Twist cmd_vel; // variable to publish
  
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 100, true);
  while(ros::ok())
  {
    cmd_vel.linear.x =20;
    cmd_vel.linear.y =20;

    pub.publish(cmd_vel);
  }

  return 0;
}