#include "clusters.hpp"

Clusters::Clusters(unsigned long int id, const pointList& points, const double& dt){

  // circles_.

  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);
  a = 1.0;
  age = 1;

  new_cluster = points;

  calcMean(points);
  previous_mean_values = mean_values;
  Circlefitting(points, mean_values);

  Tracker tracker_ukf(center_point.first, center_point.second, dt);
  this-> circle = tracker_ukf;

  populateTrackingMsgs(dt);
}

void Clusters::update(const pointList& new_points, const double& dt) {
  age++;
  previous_mean_values = mean_values;
  new_cluster = new_points;

  calcMean(new_points);
  Circlefitting(new_points, mean_values);
  
  circle.update(mean_values.first, mean_values.second, dt, new_points.size());
  
  this->calcMean(new_points);

  // populateTrackingMsgs(dt);
}

void Clusters::populateTrackingMsgs(const double& dt){
  // This function populates the Tracks msgs.
  msg_track.id = this->id;
  msg_track.odom.header.stamp = ros::Time::now();
  msg_track.odom.header.frame_id = "robot_2/base_link";
  msg_track.odom.pose.pose.position.x = cx;
  msg_track.odom.pose.pose.position.y = cy;
  msg_track.odom.twist.twist.linear.x = cvx;
  msg_track.odom.twist.twist.linear.y = cvy;

}

visualization_msgs::Marker Clusters::getVisualisationMessage() {

  visualization_msgs::Marker bb_msg;
  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = "robot_2/base_link";
  bb_msg.ns = "tracking_point";
  bb_msg.action = visualization_msgs::Marker::ADD;
  bb_msg.pose.orientation.w = 1.0;
  bb_msg.type = visualization_msgs::Marker::POINTS;
  bb_msg.id = this->id;
  //bb_msg.scale.x = 0.3; //line width
  //bb_msg.scale.x = 0.008; //line width

  bb_msg.scale.x = 0.04;
  bb_msg.scale.y = 0.04;

  bb_msg.color.g = this->g;
  bb_msg.color.b = this->b;
  bb_msg.color.r = this->r;
  bb_msg.color.a = 1.0;
  bb_msg.lifetime = ros::Duration();

  // geometry_msgs::Point p;
  // for (unsigned int i = 0; i < 4; ++i) {
  //   p.x = corner_list[i].first;  
  //   p.y = corner_list[i].second;  
  //   bb_msg.points.push_back(p);
  // }
  // p.x = corner_list[0].first;  
  // p.y = corner_list[0].second;  
  // bb_msg.points.push_back(p);

  return bb_msg;
}
// 클러스터 내에서 가장 거리가 먼 두 점을 찾는 함수
void Clusters::findFarthestPoints(const std::vector<Point>& cluster, Point& point1, Point& point2) {
  double maxDistance = 0.0;  // 최대 거리 변수 초기화
  unsigned int numPoints = cluster.size();

  // 모든 점 쌍에 대한 거리 계산 및 최대 거리 갱신
  for (unsigned int i = 0; i < numPoints - 1; ++i) {
    for (unsigned int j = i + 1; j < numPoints; ++j) {
      double distance = std::sqrt(std::pow(cluster[j].first - cluster[i].first, 2.0) +
                                  std::pow(cluster[j].second - cluster[i].second, 2.0));
      if (distance > maxDistance) {
        maxDistance = distance;
        point1 = cluster[i];
        point2 = cluster[j];
      }
    }
  }
}

void Clusters::Circlefitting(const pointList& point, Point mean_values){
  Point farthestPoint1, farthestPoint2;
  findFarthestPoints(point, farthestPoint1, farthestPoint2);
  double length = sqrt(pow(farthestPoint1.first - farthestPoint2.first, 2) + pow(farthestPoint1.second-farthestPoint2.second, 2));
  
  if (length < p_max_circle_radius_) {
    Circle circle(mean_values, length/2);
    center_point = mean_values;
    radius = length/2;
    circles_.push_back(circle);
  }
}

visualization_msgs::Marker Clusters::CircleVisualisationMessage() {
  for (unsigned int i = 0; i < circles_.size(); ++i)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "robot_2/base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "circle";
    marker.action = visualization_msgs::Marker::ADD;    
    marker.pose.orientation.w = 1.0;  
    
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x = circles_[i].center.first;
    marker.pose.position.y = circles_[i].center.second;
    marker.id = this->id;

    marker.scale.x = marker.scale.y = (circles_[i].radius)*2;
    marker.color.a = 1.0; // Don't forget to set the alpha!

    marker.color.r = 0.98;
    marker.color.g = 0.58;
    marker.color.b = 0.82;
    //only if using a MESH_RESOURCE marker type:

    return marker;
  }
}

void Clusters::calcMean(const pointList& c){

  double sum_x = 0, sum_y = 0;

  for(unsigned int i = 0; i<c.size(); ++i){

    sum_x = sum_x + c[i].first;
    sum_y = sum_y + c[i].second;
  }

    this->mean_values.first = sum_x / c.size();
    this->mean_values.second= sum_y / c.size();
}