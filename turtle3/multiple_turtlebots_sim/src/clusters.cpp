#include "clusters.hpp"


Clusters::Clusters(unsigned long int id, const pointList& points, const double& dt){
  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);

  calcMean(points);
  this->num_points = points.size();
  populateTrackingMsgs(dt);
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
void Clusters::update(const pointList& new_points, const double& dt) {
  this->calcMean(new_points);

  populateTrackingMsgs(dt);
}
void Clusters::populateTrackingMsgs(const double& dt){
  // This function populates the Tracks msgs.
  msg_track.id = this->id;
  msg_track.odom.header.stamp = ros::Time::now();

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
