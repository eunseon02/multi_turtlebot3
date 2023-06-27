#include "lidar.hpp"

Tmo::Tmo(){
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    n_private.param("euclidean_distance", euclidean_distance, 0.25);
    n_private.param("max_cluster_size", max_cluster_size, 360);
    n_private.param("threshold_distance", dth, 0.2);
    n_private.param("lidar_frame", lidar_frame, string("laser"));
    n_private.param("world_frame", world_frame, string("map"));
    
    ros::Rate r(30);
    sub_scan = n.subscribe("/robot_1/scan", 1, &Tmo::callback, this);
    pub_marker_array   = n.advertise<visualization_msgs::MarkerArray>("marker_array", 100);
    clustering_res = n.advertise<geometry_msgs::Polygon>("clustering_result", 100);
    laser_callback = n.advertise<sensor_msgs::LaserScan>("laser_call", 100);
    // vis_pub = n.advertise<visualization_msgs::Marker>("/temp_marker", 1);
}

Tmo::~Tmo(){
  
}

void Tmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){

  ROS_INFO("callback");  
  ros::Rate rate(100); 
  // if (scan_in == nullptr) {
  //       ROS_ERROR("Invalid laser scan data received");
  //       return;
  //   }

  //   if (scan_in->ranges.empty()) {
  //       ROS_ERROR("Empty laser scan data received");
  //       return;
  //   }

  // 유효한 데이터가 할당되었으므로 계속 진행

  LiDARmsg(scan_in);
  //visualizeGroupedPoints(clusters, clustering_res);
  while (ros::ok()) {
    laser_callback.publish(scan_in);
    // 루프 주기 대기
    rate.sleep();
  }

}
void Tmo::LiDARmsg(const sensor_msgs::LaserScan::ConstPtr& scan_in){

    ROS_INFO("liarmsg");
    
//-------------------
    scan = *scan_in;

    int cpoints = 0; //laser data array

    for (unsigned int i = 0; i < scan.ranges.size(); ++i){
      //inf laser count
      if(isinf(scan.ranges[i]) == 0){
          cpoints++;
        }
      }

      const int c_points = cpoints;

      int j = 0;

      vector< vector<float> > polar(c_points +1 ,vector<float>(2)); //c_points+1 for wrapping
      //polar = [
      //   [0, 0],
      //   [0, 0],
      //   [0, 0],
      //   [0, 0],
      //   [0, 0],
      //   [0, 0]
      //     ..
      //]

      for(unsigned int i = 0; i<scan.ranges.size(); ++i){
      if(!isinf(scan.ranges[i])){      // ignore inf laser data
          polar[j][0] = scan.ranges[i]; //first : 거리 값의 배열
          polar[j][1] = scan.angle_min + i*scan.angle_increment; 
          //second : laser 빔의 실제 각도들 저장^^ 
          j++;
        }
     }

    //Complete the circle
    polar[c_points] = polar[0];

    vector<pointList> point_clusters_not_transformed;
    Tmo::Clustering(point_clusters_not_transformed, polar, c_points);   
    

//--------------------update adaptive threshold distance
    vector<pointList> point_clusters;
    vector<bool> cluster_group(point_clusters.size(),false);   
    vector<bool> c_matched(clusters.size(),false);

    double euclidean[point_clusters.size()][clusters.size()]; 
    // save the euclidean distances

    //Finding mean coordinates of group and associating with cluster Objects
    double mean_x = 0, mean_y = 0;

    for(unsigned int g = 0; g<point_clusters.size();++g){
      double sum_x = 0, sum_y = 0;
    
      for(unsigned int l =0; l<point_clusters[g].size(); l++){
        sum_x = sum_x + point_clusters[g][l].first;
        sum_y = sum_y + point_clusters[g][l].second;
        std::ostringstream oss;
        oss << point_clusters[g][l].first << ", " << point_clusters[g][l].second;
        ROS_INFO("%s", oss.str().c_str());
      }
      mean_x = sum_x / point_clusters[g].size();
      mean_y = sum_y / point_clusters[g].size();

      for(unsigned int c=0;c<clusters.size();++c){
        euclidean[g][c] = abs( mean_x - clusters[c].meanX()) + abs(mean_y - clusters[c].meanY()); 
      }
    }
//-------------------------------
//-----------------------------Tracking


    // 점들 간 최소 거리 < 클러스터 내부 점들 간 최대 거리인 경우 같은 클러스터
    vector<pair <int,int>> pairs;
    for(unsigned int c=0; c<clusters.size();++c){
      unsigned int position;
      double min_distance = euclidean_distance;

      for(unsigned int g=0; g<point_clusters.size();++g){      // smallest euclidean distance
      if(euclidean[g][c] < min_distance){ 
            min_distance = euclidean[g][c];
            position = g;
      }
      }
      if(min_distance < euclidean_distance){ //point inside cluster  
        cluster_group[position] = true, c_matched[c] = true;
        pairs.push_back(pair<int,int>(c,position));
      }
  
    }

    //Update Clusters
    #pragma omp parallel for
    for(unsigned int p=0; p<pairs.size();++p){
      clusters[pairs[p].first].update(point_clusters[pairs[p].second]);
    }
       
    //Delete Clusters
    unsigned int o=0;
    unsigned int p = clusters.size();
    while(o<p){
      if(c_matched[o] == false){
        //순서 맨 뒤로 바꿔서 제거
        std::swap(clusters[o], clusters.back());
        clusters.pop_back();

        std::swap(c_matched[o], c_matched.back());
        c_matched.pop_back();

        o--;
        p--; //제거할 point
      }
    o++;
    }

    // Initialisation of new Cluster Objects
    for(unsigned int i=0; i<point_clusters.size();++i){
      if(cluster_group[i] == false && point_clusters[i].size()< max_cluster_size){
        Clusters cl(cclusters, point_clusters[i]);
        cclusters++;
        clusters.push_back(cl);
      } 
    }
    

    // ROS_INFO("vis_ini");
    // //cluster info
    // for (size_t i = 0; i < point_clusters.size(); ++i) {
    //   ROS_INFO_STREAM("Cluster " << i << ":");
    //   for (const auto& point : point_clusters[i]) {
    //     ROS_INFO_STREAM(" Point: x=" << point.first << ", y=" << point.second);
    //   }
    // }


    visualizeGroupedPoints(Tmo::point_clusters);


}

void Tmo::Clustering(vector<pointList> &clusters, vector< vector<float> > &polar, const int c_points){

ROS_INFO("clustering");
///----------------------------------
  //////////////Adaptive Breakpoint Detector Algorithm//////////////////
  ///////////////////////////////////////////////////////////////////// 
  float d;  //adaptive threshold distance

  vector<bool> clustered1(c_points+1 ,false); //change to true when it is the first of the cluster
  vector<bool> clustered2(c_points+1 ,false); // another cluster

  float l = 45; // λ:acceptable angle
  l = l * 0.0174532;  // degree to radian conversion
  int cluster_ns = 1; // give each cluster name

  const float s = 0;   // σr is the standard deviation of the noise of the distance measure
  for (unsigned int i=0; i < c_points ; ++i){
      double dtheta = polar[i+1][1]- polar[i][1];
      double adaptive = min(polar[i][0],polar[i+1][0]) * (sin(dth)) / (sin(l - (dth))) + s; //Dthreshold
      d = sqrt( pow(polar[i][0],2) + pow(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*cos(polar[i+1][1] - polar[i][1]));
      if(d<dth) {  // included in cluster 
        //polar[i].push_back(cluster_ns);
        clustered1[i] = true;   
        clustered2[i+1] = true;
      }
  }
////////////////////////////////////////////////////////////////////////////////////////////////
  clustered2[0] = clustered2[c_points];

  vector<int> begin; //first index of a cluster
  vector<int> nclus; //number of clustered points
  int i =0;
  bool flag = true; // flag for not going back through the stack 

  while(i<c_points && flag==true){

      if (clustered1[i] == true && clustered2[i] == false && flag == true){ //first element of cluster
          begin.push_back(i);
          nclus.push_back(1);
          while(clustered2[i+1] == true && clustered1[i+1] == true ){
              i++;
              ++nclus.back(); //return vector last element
              if(i==c_points-1 && flag == true){
                  i = -1;
                  flag = false;//last point까지 다 함
              }
          }
          ++nclus.back();
      }
      i++;
      }
      // last point is new cluster 
      if(clustered1[c_points-1]== true and clustered2[c_points-1] == false){
          begin.push_back(c_points-1);
          nclus.push_back(1);
          i = 0;
          while(clustered2[i] == true && clustered1[i] == true ){
              i++;
              ++nclus.back();
          }
      }

  polar.pop_back(); //remove the wrapping element
  int len = polar.size();

  for(unsigned int i=0; i<begin.size(); ++i){

  pointList cluster;

  double x,y;
  int j =begin[i];
  bool fl = true; // flag for not going back through the stack 

  while (j<nclus[i]+begin[i]){
      if(j== len && fl == true) fl = false;
      if (fl == true)
      {
      x = polar[j][0] * cos(polar[j][1]);       //x = r × cos( θ )
      y = polar[j][0] * sin(polar[j][1]);       //y = r × sin( θ )
      }
      else{
      x = polar[j-len][0] *cos(polar[j-len][1]); //x = r × cos( θ )
      y = polar[j-len][0] *sin(polar[j-len][1]); //y = r × sin( θ ) 
      }
      cluster.push_back(Point(x, y));
      
      ++j;
  }
  clusters.push_back(cluster);
  Tmo::point_clusters = clusters;


  //new cluster


  //delete cluster


  //add points



  //cluster info
  // for (size_t i = 0; i < clusters.size(); ++i) {
  //   ROS_INFO_STREAM("Cluster " << i << ":");
  //   for (const auto& point : clusters[i]) {
  //     ROS_INFO_STREAM(" Point: x=" << point.first << ", y=" << point.second);
  //   }
  // }

  //cluster info
  // ROS_INFO("Cluster %d:", i + 1);
  // for (const auto& point : cluster) {
  //   ROS_INFO("x: %f, y: %f", point.first, point.second);
  // }
  // }

  //visualizeGroupedPoints(clusters);
}
}

void Tmo::visualizeGroupedPoints(const std::vector<pointList>& point_clusters){
  ROS_INFO("pub_vist");
  //ROS_INFO("pub_viswswt");
  //ROS_INFO(point_clusters.size());

  for (size_t i = 0; i < point_clusters.size(); ++i) {
    ROS_INFO_STREAM("Cluster22 " << i << ":");
    for (const auto& point : point_clusters[i]) {
      ROS_INFO_STREAM(" Point22: x=" << point.first << ", y=" << point.second);
      //ros::Duration(5).sleep();
    }
  }

  // if(point_clusters.size() == 0) ROS_INFO_STREAM("there's no point in clusters!");


  // geometry_msgs::Polygon polygon_msg;

  // for (const auto& polygon : point_clusters)
  // {
  //   //ROS_INFO("GET IN");
  //   for (const auto& point : polygon)
  //   {
  //     geometry_msgs::Point32 msg_point;
  //     msg_point.x = point.first;
  //     msg_point.y = point.second;
  //     // z, w 값도 필요한 경우 설정 가능
  //     polygon_msg.points.push_back(msg_point);
  //     //ROS_INFO("msg_point: (%f, %f)", msg_point.x, msg_point.y);
  //   }
  // }
  // clustering_res.publish(polygon_msg);


  // int num_subscribers = clustering_res.getNumSubscribers();
  // ROS_INFO("Number of subscribers: %d", num_subscribers);
  //ros::Rate r(30);

  // while(ros::ok()){
  visualization_msgs::MarkerArray marker_array;

  for(unsigned int i=0; i<point_clusters.size(); ++i){

    visualization_msgs::Marker gpoints;

    gpoints.header.frame_id = "robot_1/base_link";
    gpoints.header.stamp = ros::Time::now();
    gpoints.ns = "clustered_point";
    gpoints.action = visualization_msgs::Marker::ADD;
    gpoints.pose.orientation.w = 1.0;

    gpoints.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    gpoints.scale.x = 0.04;
    gpoints.scale.y = 0.04;
    gpoints.id = cg;

    cg++;
    gpoints.color.g = rand() / double(RAND_MAX);
    gpoints.color.b = rand() / double(RAND_MAX);
    gpoints.color.r = rand() / double(RAND_MAX);
    gpoints.color.a = 1.0; // 알파 값을 1.0-불투명도
    gpoints.lifetime = ros::Duration();

    for(unsigned int j=0; j<point_clusters[i].size(); ++j){
      geometry_msgs::Point p;
      p.x = point_clusters[i][j].first;
      p.y = point_clusters[i][j].second;
      p.z = 0;
      gpoints.points.push_back(p);
    }
    marker_array.markers.push_back(gpoints);
  }
  pub_marker_array.publish(marker_array);
    

  //ros::spinOnce();
  //r.sleep();
  //ROS_INFO("now");

}

