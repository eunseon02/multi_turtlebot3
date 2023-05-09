#include "ros/ros.h"
#include "cmd_vel_pub/Cmd_vel_msg.h"
  
#define PUB_NODE_NAME "/robot1/cmd_vel" // name of node
#define TOPIC_NAME "cmd_vel_topic" // name of topic : cmd_vel_pub
 
float vel_x, vel_y;
  
int main(int argc, char **argv){
  ros::init(argc, argv, PUB_NODE_NAME);
  ros::NodeHandle nh;
  geometry_msgs::Twist cmd_vel; // variable to publish
  
  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_NAME, 100, true);
  ros::Rate loop_rate(0.3);
  
    cmd_vel.linear.x =0;
    cmd_vel.linear.y =0;
  
  while (ros::ok()){
  
    std::cout << "input velocity" << std::endl;
    std::cin >> vel_x >> vel_y;
  
    float current_x = cmd_vel.linear.x;
    float current_y = cmd_vel.linear.y;
  
 
    loop_rate.sleep();// Goes to sleep according to the loop rate defined above.
  
  }
  return 0;
}