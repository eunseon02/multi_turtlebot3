#include "lidar.hpp"

Tmo::Tmo()
{
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  n_private.param("euclidean_distance", euclidean_distance, 0.25);
  n_private.param("max_cluster_size", max_cluster_size, 360);
  n_private.param("threshold_distance", dth, 0.2);
  n_private.param("lidar_frame", lidar_frame, string("laser"));
  n_private.param("world_frame", world_frame, string("map"));
  n_private.param("pub_markers", p_marker_pub, true);
  n_private.param("use_split_and_merge", p_use_split_and_merge_, true);
  n_private.param("min_group_points", p_min_group_points_, 5);
  n_private.param("max_group_distance", p_max_group_distance_, 0.1);
  n_private.param("distance_proportion", p_distance_proportion_, 0.00628);
  n_private.param("max_split_distance", p_max_split_distance_, 0.2);

  sub_scan = n.subscribe("/robot_2/scan", 1, &Tmo::callback, this);
  pub_marker_array = n.advertise<visualization_msgs::MarkerArray>("marker_array_t", 100);
  clustering_res = n.advertise<geometry_msgs::Polygon>("clustering_result", 100);
  laser_callback = n.advertise<sensor_msgs::LaserScan>("laser_call", 100);
  // obstacles_pub_ = n.advertise<multiple_turtlebots_sim::Obstacles>("raw_obstacles", 10);
  // vis_pub = n.advertise<visualization_msgs::Marker>("/temp_marker", 1);
}

Tmo::~Tmo()
{
}

void Tmo::callback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{

  ROS_INFO("callback");
  // 유효한 데이터가 할당되었으므로 계속 진행
  LiDARmsg(scan_in);
}
void Tmo::LiDARmsg(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
  // marker 초기화
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markera;
  marker.action = 3;
  markera.markers.push_back(marker);
  pub_marker_array.publish(markera);

  // markera.markers.clear();
  // pub_marker_array.publish(markera);

  ROS_INFO("liarmsg");

  //-------------------
  scan = *scan_in;

  // 초당 프레임
  dt = 0.08;
  int cpoints = 0; // laser data array

  for (unsigned int i = 0; i < scan.ranges.size(); ++i)
  {
    // inf laser count
    if (isinf(scan.ranges[i]) == 0)
    {
      cpoints++;
    }
  }

  const int c_points = cpoints;
  int j = 0;

  vector<vector<float>> polar(c_points + 1, vector<float>(2)); // c_points+1 for wrapping
  // polar = [
  //    [0, 0],
  //    [0, 0],
  //    [0, 0],
  //    [0, 0],
  //    [0, 0],
  //    [0, 0]
  //      ..
  //]

  for (unsigned int i = 0; i < scan.ranges.size(); ++i)
  {
    if (!isinf(scan.ranges[i]))
    {                               // ignore inf laser data
      polar[j][0] = scan.ranges[i]; // first : 거리 값의 배열
      polar[j][1] = scan.angle_min + i * scan.angle_increment;
      // second : laser 빔의 실제 각도들 저장^^
      j++;
    }
  }

  // Complete the circleas
  polar[c_points] = polar[0];

  vector<pointList> point_clusters_not_transformed;
  Clustering(point_clusters_not_transformed, polar, c_points);

  vector<pointList> point_clusters;
  for (unsigned int i = 0; i < point_clusters_not_transformed.size(); ++i)
  {
    pointList point_cluster;
    point_clusters.push_back(point_clusters_not_transformed[i]);
  }

  // cluster info
  //  for (size_t i = 0; i < point_clusters.size(); ++i) {
  //    ROS_INFO_STREAM("Cluster " << i << ":");
  //    for (const auto& point : point_clusters) {
  //      ROS_INFO_STREAM(" Point: x=" << point.first << ", y=" << point.second);
  //    }
  //  }

  // for (size_t i = 0; i < clusters.size(); ++i) {
  //   ROS_INFO_STREAM("Cluster " << i << ":");
  //   for (const auto& point : clusters) {
  //     ROS_INFO_STREAM(" Point: x=" << point.first << ", y=" << point.second);
  //   }
  // }

  //--------------------update adaptive threshold distance
  vector<bool> cluster_group(point_clusters.size(), false);
  vector<bool> c_matched(clusters.size(), false);

  double euclidean[point_clusters.size()][clusters.size()];
  // save the euclidean distances

  // Finding mean coordinates of group and associating with cluster Objects
  double mean_x = 0, mean_y = 0;

  for (unsigned int g = 0; g < point_clusters.size(); ++g)
  {
    double sum_x = 0, sum_y = 0;

    for (unsigned int l = 0; l < point_clusters[g].size(); l++)
    {
      sum_x = sum_x + point_clusters[g][l].first;
      sum_y = sum_y + point_clusters[g][l].second;
      std::ostringstream oss;
      oss << point_clusters[g][l].first << ", " << point_clusters[g][l].second;
      // ROS_INFO("%s", oss.str().c_str());
    }
    mean_x = sum_x / point_clusters[g].size();
    mean_y = sum_y / point_clusters[g].size();

    for (unsigned int c = 0; c < clusters.size(); ++c)
    {
      euclidean[g][c] = abs(mean_x - clusters[c].meanX()) + abs(mean_y - clusters[c].meanY());
    }
  }

  if (point_clusters.empty())
  {
    ROS_INFO("point_clusters is empty");
  }
  else
  {
    ROS_INFO("point_clusters has %zu elements", point_clusters.size());
  }

  if (clusters.empty())
  {
    ROS_INFO("clusters is empty");
  }
  else
  {
    ROS_INFO("clusters has %zu elements", clusters.size());
  }

  for (const auto &cluster : clusters)
  {
    float r = cluster.r;
    float g = cluster.g;
    float b = cluster.b;
    ROS_INFO("Cluster color: r=%.2f, g=%.2f, b=%.2f", r, g, b);
  }

  //-------------------------------
  //-----------------------------Tracking
  // 점들 간 최소 거리 < 클러스터 내부 점들 간 최대 거리인 경우 같은 클러스터
  vector<pair<int, int>> pairs;
  for (unsigned int c = 0; c < clusters.size(); ++c)
  {
    unsigned int position;
    double min_distance = euclidean_distance;

    for (unsigned int g = 0; g < point_clusters.size(); ++g)
    { // smallest euclidean distance
      if (euclidean[g][c] < min_distance)
      {
        min_distance = euclidean[g][c];
        position = g;
      }
    }
    if (min_distance < euclidean_distance)
    { // point inside cluster
      cluster_group[position] = true, c_matched[c] = true;
      pairs.push_back(pair<int, int>(c, position));
    }
  }

// Update Clusters
#pragma omp parallel for
  for (unsigned int p = 0; p < pairs.size(); ++p)
  {
    clusters[pairs[p].first].update(point_clusters[pairs[p].second], dt);
  }

  // Delete Clusters
  unsigned int o = 0;
  unsigned int p = clusters.size();
  while (o < p)
  {
    if (c_matched[o] == false)
    {
      // 순서 맨 뒤로 바꿔서 제거
      std::swap(clusters[o], clusters.back());
      clusters.pop_back();

      std::swap(c_matched[o], c_matched.back());
      c_matched.pop_back();

      o--;
      p--; // 제거할 point
    }
    o++;
  }

  // Initialisation of new Cluster Objects
  for (unsigned int i = 0; i < point_clusters.size(); ++i)
  {
    if (cluster_group[i] == false && point_clusters.size() < max_cluster_size)
    {
      Clusters cl(cclusters, point_clusters[i], dt);
      cclusters++;
      clusters.push_back(cl);
    }
  }

  ROS_INFO("vis_ini");

  // Visualizations and msg publications
  visualization_msgs::MarkerArray marker_array;
  // multiple_turtlebots_sim::TrackArray msg_track;
  // for (unsigned int i =0; i<clusters.size();i++){
  //   // msg_track.tracks.push_back(clusters[i].msg_track);
  //   marker_array.markers.push_back(clusters[i].getVisualisationMessage());
  //   pub_marker_array.publish(marker_array);
  // }

  // for (unsigned int i =0; i<clusters.size();i++){
  //   detectSegments(clusters[i]);
  // }
  visualizeGroupedPoints(point_clusters);
}

void Tmo::Clustering(vector<pointList> &clusters, vector<vector<float>> &polar, const int c_points)
{

  ROS_INFO("clustering");
  //////////////Adaptive Breakpoint Detector Algorithm//////////////////
  /////////////////////////////////////////////////////////////////////
  float d; // adaptive threshold distance

  vector<bool> clustered1(c_points + 1, false); // change to true when it is the first of the cluster
  vector<bool> clustered2(c_points + 1, false); // another cluster

  float l = 45;       // λ:acceptable angle
  l = l * 0.0174532;  // degree to radian conversion
  int cluster_ns = 1; // give each cluster name

  const float s = 0; // σr is the standard deviation of the noise of the distance measure
  for (unsigned int i = 0; i < c_points; ++i)
  {
    double dtheta = polar[i + 1][1] - polar[i][1];
    double adaptive = min(polar[i][0], polar[i + 1][0]) * (sin(dth)) / (sin(l - (dth))) + s; // Dthreshold
    d = sqrt(pow(polar[i][0], 2) + pow(polar[i + 1][0], 2) - 2 * polar[i][0] * polar[i + 1][0] * cos(polar[i + 1][1] - polar[i][1]));
    if (d < dth)
    { // included in cluster
      // polar[i].push_back(cluster_ns);
      clustered1[i] = true;
      clustered2[i + 1] = true;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
  clustered2[0] = clustered2[c_points];

  vector<int> begin; // first index of a cluster
  vector<int> nclus; // number of clustered points
  int i = 0;
  bool flag = true; // flag for not going back through the stack

  while (i < c_points && flag == true)
  {

    if (clustered1[i] == true && clustered2[i] == false && flag == true)
    { // first element of cluster
      begin.push_back(i);
      nclus.push_back(1);
      while (clustered2[i + 1] == true && clustered1[i + 1] == true)
      {
        i++;
        ++nclus.back(); // return vector last element
        if (i == c_points - 1 && flag == true)
        {
          i = -1;
          flag = false; // last point까지 다 함
        }
      }
      ++nclus.back();
    }
    i++;
  }
  // last point is new cluster
  if (clustered1[c_points - 1] == true and clustered2[c_points - 1] == false)
  {
    begin.push_back(c_points - 1);
    nclus.push_back(1);
    i = 0;
    while (clustered2[i] == true && clustered1[i] == true)
    {
      i++;
      ++nclus.back();
    }
  }

  polar.pop_back(); // remove the wrapping element
  int len = polar.size();

  for (unsigned int i = 0; i < begin.size(); ++i)
  {

    pointList cluster;

    double x, y;
    int j = begin[i];
    bool fl = true; // flag for not going back through the stack

    while (j < nclus[i] + begin[i])
    {
      if (j == len && fl == true)
        fl = false;
      if (fl == true)
      {
        x = polar[j][0] * cos(polar[j][1]); // x = r × cos( θ )
        y = polar[j][0] * sin(polar[j][1]); // y = r × sin( θ )
      }
      else
      {
        x = polar[j - len][0] * cos(polar[j - len][1]); // x = r × cos( θ )
        y = polar[j - len][0] * sin(polar[j - len][1]); // y = r × sin( θ )
      }
      cluster.push_back(Point(x, y));

      ++j;
    }
    clusters.push_back(cluster);
    // ROS_INFO(typeid(clusters));

    // //cluster info
    // for (size_t i = 0; i < clusters.size(); ++i) {
    //   ROS_INFO_STREAM("Cluster " << i << ":");
    //   for (const auto& point : clusters) {
    //     ROS_INFO_STREAM(" Point: x=" << point.first << ", y=" << point.second);
    //   }
    // }

    // cluster info
    //  ROS_INFO("Cluster %d:", i + 1);
    //  for (const auto& point : cluster) {
    //    ROS_INFO("x: %f, y: %f", point.first, point.second);
    //  }
  }
}

void Tmo::visualizeGroupedPoints(const std::vector<pointList> &point_clusters)
{
  ROS_INFO("pub_vist");

  for (size_t i = 0; i < point_clusters.size(); ++i)
  {
    ROS_INFO_STREAM("Cluster22 " << i << ":");
    for (const auto &point : point_clusters[i])
    {
      ROS_INFO_STREAM(" Point22: x=" << point.first << ", y=" << point.second);
    }
  }

  // if(point_clusters.size() == 0) ROS_INFO_STREAM("there's no point in clusters!");

  visualization_msgs::MarkerArray marker_array;

  for (unsigned int i = 0; i < point_clusters.size(); ++i)
  {

    visualization_msgs::Marker gpoints; // 화이팅 최은선 멋지다 최은선

    gpoints.header.frame_id = "robot_2/base_link";
    gpoints.header.stamp = ros::Time::now();

    gpoints.ns = "clustered_point";
    gpoints.action = visualization_msgs::Marker::ADD;
    gpoints.pose.orientation.w = 1.0;

    gpoints.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    gpoints.scale.x = 0.08;
    gpoints.scale.y = 0.08;
    gpoints.id = cg;

    cg++;
    // gpoints.color.g = rand() / double(RAND_MAX);
    // gpoints.color.b = rand() / double(RAND_MAX);
    // gpoints.color.r = rand() / double(RAND_MAX);

    gpoints.color.g = 255;
    gpoints.color.b = 100;
    gpoints.color.r = 100;
    
    gpoints.color.a = 1.0; // 알파 값을 1.0-불투명도
    gpoints.lifetime = ros::Duration();

    for (unsigned int j = 0; j < point_clusters.size(); ++j)
    {
      geometry_msgs::Point p;
      if (point_clusters[i][j].first < 1 && point_clusters[i][j].second < 1)
      {
        p.x = point_clusters[i][j].first;
        p.y = point_clusters[i][j].second;
      }
      else
      {
        p.x = 0.0;
        p.y = 0.0;
      }
      p.z = 0;
      ROS_INFO("p.x : %f, p.y : %f", p.x, p.y);
      gpoints.points.push_back(p);
    }
    marker_array.markers.push_back(gpoints);
  }
  pub_marker_array.publish(marker_array);
}

// void Tmo::detectSegments(const Clusters& cluster) {

//   // 초당 프레임
//   dt = 0.08;

//   if (cluster.num_points < p_min_group_points_)
//     return;

//   Segment segment(*cluster.begin, *cluster.end);  // Use Iterative End Point Fit

//   if (p_use_split_and_merge_)
//     segment = fitSegment(clusters);

//   PointIterator set_divider;
//   double max_distance = 0.0;
//   double distance     = 0.0;

//   int split_index = 0; // Natural index of splitting point (counting from 1)
//   int point_index = 0; // Natural index of current point in the set

//   // Seek the point of division
//   for (PointIterator point = cluster.begin; point != cluster.end; ++point) {
//     ++point_index;

//     if ((distance = segment.distanceTo(*point)) >= max_distance) {
//       double r = length(*point);

//       if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
//         max_distance = distance;
//         set_divider = point;
//         split_index = point_index;
//       }
//     }
//   }

//   // Split the set only if the sub-groups are not 'small'
//   if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < cluster.num_points - p_min_group_points_) {
//     set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point for each group

//     // Clusters subset1(), subset2();
//     // subset1.begin = point_set.begin;
//     // subset1.end = set_divider;
//     // subset1.num_points = split_index;
//     // subset1.is_visible = point_set.is_visible;

//     // subset2.begin = ++set_divider;
//     // subset2.end = point_set.end;
//     // subset2.num_points = point_set.num_points - split_index;
//     // subset2.is_visible = point_set.is_visible;

//   // Subset 1
//   pointList subset1_points(cluster.begin, set_divider);
//   Clusters subset1(cluster.id, subset1_points, dt);

//   // Subset 2
//   pointList subset2_points(set_divider, cluster.end);
//   Clusters subset2(cluster.id, subset2_points, dt);

//   detectSegments(subset1);
//   detectSegments(subset2);
//   } else {  // Add the segment
//     if (!p_use_split_and_merge_)
//       segment = fitSegment(clusters);

//     segments_.push_back(segment);
//   }
// }

// void Tmo::publishObstacles()
// {
//   const std::vector<pointList> point_clusters ;

//   //clusters -> segments
//   for (size_t i = 0; i < clusters.size(); ++i) {
//     //  Segment segment(*point_set.begin, *point_set.end);
//     Segment segment(*clusters[i].begin, *clusters[i].end);  // Use Iterative End Point Fit
//     segment = fitSegment(clusters);
//     segments_.push_back(segment);
//   }

//   ///////
//   multiple_turtlebots_sim::Obstacles obstacles_msg;
//   obstacles_msg.header.stamp = ros::Time::now();
//   tf::StampedTransform transform;

//   for (size_t i = 0; i < segments_.size(); ++i) {
//     const Segment& s = segments_[i];

//   }
//   for (size_t i = 0; i < segments_.size(); ++i) {
//     const Segment& s = segments_[i];
//     multiple_turtlebots_sim::SegmentObstacle segment;

//     segment.first_point.x = s.first_point.first;
//     segment.first_point.y = s.first_point.second;
//     segment.last_point.x = s.last_point.first;
//     segment.last_point.y = s.last_point.second;

//     obstacles_msg.segments.push_back(segment);
//   }
//   obstacles_pub_.publish(obstacles_msg);
// }

// Segment Tmo::fitSegment(const std::vector<Clusters>&  clusters) {
//   static int num_calls = 0;
//   num_calls++;

//   int N = 0;
//   for (Clusters cc : clusters)
//     N += cc.num_points;

//   assert(N >= 2);

//   arma::mat input  = arma::mat(N, 2).zeros();  // [x_i, y_i]
//   arma::vec output = arma::vec(N).ones();      // [-C]
//   arma::vec params = arma::vec(2).zeros();     // [A ; B]

//   int n = 0;
//   for (Clusters cc : clusters) {
//     PointIterator point = cc.begin;
//     for (int i = 0; i < cc.num_points; ++i) {
//       input(i + n, 0) = point->first;
//       input(i + n, 1) = point->second;
//       std::advance(point, 1);
//     }

//     n += cc.num_points;
//   }

//   // Find A and B coefficients from linear regression (assuming C = -1.0)
//   params = arma::pinv<arma::mat>(input) * output;

//   double A, B, C;
//   A = params(0);
//   B = params(1);
//   C = -1.0;

//   // Find end points
//   Point p1 = *clusters.front().begin;
//   Point p2 = *clusters.back().end;

//   Segment segment(p1, p2);
//   segment.clusters = clusters;

//   double D = (A * A + B * B);

//   // Project end points on the line
//   if (D > 0.0) {
//     Point projected_p1, projected_p2;

//     projected_p1.first = ( B * B * p1.first - A * B * p1.second - A * C) / D;
//     projected_p1.second = (-A * B * p1.first + A * A * p1.second - B * C) / D;

//     projected_p2.first = ( B * B * p2.first - A * B * p2.second - A * C) / D;
//     projected_p2.second = (-A * B * p2.first + A * A * p2.second - B * C) / D;

//     segment.first_point = projected_p1;
//     segment.last_point = projected_p2;
//   }

//    return segmnet;
// }
