 #include <ros/ros.h>
#include <geometry_msgs/Polygon.h>


void polygonCallback(const geometry_msgs::Polygon::ConstPtr& msg)
{
  // 수신된 메시지 처리
  // 여기에 원하는 동작을 구현하세요
  ROS_INFO_STREAM("subBBB");
  ROS_INFO("Received Polygon Message");
  std::cout<<"5555"<<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polygon_subscribe_node");
  ros::NodeHandle nh("~");

  // ...

  // Subscriber 생성
  ros::Subscriber polygon_sub = nh.subscribe("test_clustering_result", 1000, polygonCallback);
  ROS_INFO("sub");
  // ...

  ros::spin();

  return 0;
}
