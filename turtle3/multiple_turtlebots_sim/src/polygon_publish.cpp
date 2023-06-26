#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <random>
#include <iostream>

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

void visualizeGroupedPoints(const std::vector<pointList>& point_clusters, ros::Publisher& clustering_res)
{
  ROS_INFO("pub");

  geometry_msgs::Polygon polygon_msg;
  for (const auto& polygon : point_clusters)
  {
    for (const auto& point : polygon)
    {
      geometry_msgs::Point32 msg_point;
      msg_point.x = point.first;
      msg_point.y = point.second;
      // z, w 값도 필요한 경우 설정 가능
      
      polygon_msg.points.push_back(msg_point);
    
      ROS_INFO("msg_point: (%f, %f)", msg_point.x, msg_point.y);

    }
  }

  // 메시지 publish
  clustering_res.publish(polygon_msg);
  
  //int num_subscribers = clustering_res.getNumSubscribers();
  //ROS_INFO("Number of subscribers: %d", num_subscribers);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polygon_publish_node");
  ros::NodeHandle nh;

  // Publisher 생성
  ros::Publisher clustering_res = nh.advertise<geometry_msgs::Polygon>("test_clustering_result", 1000);

  // point_clusters 데이터 생성
  std::vector<pointList> point_clusters;
  // point_clusters에 데이터 추가

  // 랜덤 넘버 생성기 초기화
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> distribution(-10.0, 10.0); // x, y 좌표 범위 설정

  // 랜덤 데이터 생성 및 point_clusters에 추가
  for (int i = 0; i < 5; ++i)
  {
    pointList polygon;
    for (int j = 0; j < 10; ++j)
    {
      double x = distribution(gen);
      double y = distribution(gen);
      polygon.push_back(std::make_pair(x, y));
    }

    point_clusters.push_back(polygon);

    // Output a double value to ros_info
    double value = polygon[0].first;  // Change the index based on your requirement
    //ROS_INFO("Double value: %f", value);
  }

  ros::Rate rate(10);  // 10Hz 주기로 메시지 발행

  while (ros::ok()) {
    // visualizeGroupedPoints 함수 호출
    visualizeGroupedPoints(point_clusters, clustering_res);

    // 루프 주기 대기
    rate.sleep();
  }

  return 0;
}
