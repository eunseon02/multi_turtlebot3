#include "lidar.hpp"

Tmo::Tmo(){
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    n_private.param("euclidean_distance", euclidean_distance, 0.25);
    n_private.param("max_cluster_size", max_cluster_size, 360);
    n_private.param("threshold_distance", dth, 0.2);
    n_private.param("lidar_frame", lidar_frame, string("laser"));
    n_private.param("world_frame", world_frame, string("map"));
    
    //ros::Rate r(30);
    sub_scan = n.subscribe("/tb3_0/scan", 1, &Tmo::callback, this);
    pub_marker_array   = n.advertise<visualization_msgs::MarkerArray>("/marker_array", 10);
    pub_marker  = n.advertise<visualization_msgs::Marker>("/marker", 10);
}

Tmo::~Tmo(){
  
}

void Tmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){

  // delete all Markers 
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markera;
  marker.action =3;
  markera.markers.push_back(marker);
  pub_marker_array.publish(markera);

  //if(tf_listener.canTransform(world_frame, lidar_frame, ros::Time())){
    
    vector<pointList> point_clusters_not_transformed;
    Tmo::Clustering(scan_in, point_clusters_not_transformed);   

    vector<pointList> point_clusters;
    vector<bool> g_matched(point_clusters.size(),false);   // The Group has been matched with a Cluster
    vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group

    double euclidean[point_clusters.size()][clusters.size()]; 
    // save the euclidean distances

    //Finding mean coordinates of group and associating with cluster Objects
    double mean_x = 0, mean_y = 0;

    for(unsigned int g = 0; g<point_clusters.size();++g){
      double sum_x = 0, sum_y = 0;
    
      for(unsigned int l =0; l<point_clusters[g].size(); l++){
        sum_x = sum_x + point_clusters[g][l].first;
        sum_y = sum_y + point_clusters[g][l].second;
      }
      mean_x = sum_x / point_clusters[g].size();
      mean_y = sum_y / point_clusters[g].size();

      for(unsigned int c=0;c<clusters.size();++c){
        euclidean[g][c] = abs( mean_x - clusters[c].meanX()) + abs(mean_y - clusters[c].meanY()); 
      }
    }

    //Find the smallest euclidean distance and associate if smaller than the threshold 
    vector<pair <int,int>> pairs;
    for(unsigned int c=0; c<clusters.size();++c){
      unsigned int position;
      double min_distance = euclidean_distance;
      for(unsigned int g=0; g<point_clusters.size();++g){
        if(euclidean[g][c] < min_distance){
        min_distance = euclidean[g][c];
        position = g;
      }
      }
      if(min_distance < euclidean_distance){
        g_matched[position] = true, c_matched[c] = true;
        pairs.push_back(pair<int,int>(c,position));
      }
    }
    ROS_INFO("vis");
    visualiseGroupedPoints(point_clusters);
    
  // }
  // else{ //If the tf is not possible init all states at 0
  //   ROS_WARN_DELAYED_THROTTLE(1 ,"No transform could be found between %s and %s", lidar_frame.c_str(), world_frame.c_str());
  // };


}

void Tmo::Clustering(const sensor_msgs::LaserScan::ConstPtr& scan_in, vector<pointList> &clusters){

    scan = *scan_in;

    int cpoints = 0; //

    //Find the number of non inf laser scan values and save them in c_points
    for (unsigned int i = 0; i < scan.ranges.size(); ++i){
    if(isinf(scan.ranges[i]) == 0){
        cpoints++;
    }
    }
    const int c_points = cpoints;

    int j = 0;
    vector< vector<float> > polar(c_points +1 ,vector<float>(2)); //c_points+1 for wrapping
    for(unsigned int i = 0; i<scan.ranges.size(); ++i){
    if(!isinf(scan.ranges[i])){
        polar[j][0] = scan.ranges[i]; //first column is the range 
        polar[j][1] = scan.angle_min + i*scan.angle_increment; //second angle in rad
        j++;
      }
    }

    //Complete the circle
    polar[c_points] = polar[0];

    //adaptive threshold distance
    float d;

    //There are two flags, since two consecutive points can belong to two independent clusters
    vector<bool> clustered1(c_points+1 ,false); //change to true when it is the first of the cluster
    vector<bool> clustered2(c_points+1 ,false); // change to true when it is clustered by another one

    float l = 45; // λ:acceptable angle
    l = l * 0.0174532;   // degree to radian conversion;
    const float s = 0;   // σr is the standard deviation of the noise of the distance measure
    for (unsigned int i=0; i < c_points ; ++i){
        double dtheta = polar[i+1][1]- polar[i][1];
        double adaptive = min(polar[i][0],polar[i+1][0]) * (sin(dth)) / (sin(l - (dth))) + s; //Dthreshold
        d = sqrt( pow(polar[i][0],2) + pow(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*cos(polar[i+1][1] - polar[i][1]));
        if(d<dth) {
            clustered1[i] = true; //both points belong to clusters
            clustered2[i+1] = true;}
    }

    clustered2[0] = clustered2[c_points];

    //Going through the points and finding the beginning of clusters and number of points
    vector<int> begin; //saving the first index of a cluster
    vector<int> nclus; //number of clustered points
    int i =0;
    bool flag = true; // flag for not going back through the stack 

    while(i<c_points && flag==true){

        if (clustered1[i] == true && clustered2[i] == false && flag == true){
            begin.push_back(i);
            nclus.push_back(1);
            while(clustered2[i+1] == true && clustered1[i+1] == true ){
                i++;
                ++nclus.back();
                if(i==c_points-1 && flag == true){
                    i = -1;
                    flag = false;
                }
            }
            ++nclus.back();//take care of 0 1 flags - last of the cluster
        }
        i++;
        }
        // take care of last point being beginning of cluster
        if(clustered1[cpoints-1]== true and clustered2[c_points-1] == false){
            begin.push_back(cpoints-1);
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
    }
}
void Tmo::visualiseGroupedPoints(const vector<pointList>& point_clusters){

  ros::Rate r(30);
  while (ros::ok())
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker gpoints, marker;
    gpoints.header.frame_id = "base_map";
    marker.header.frame_id = "base_map";
    gpoints.header.stamp = marker.header.stamp = ros::Time::now();
    gpoints.ns = "clustered_point";
    marker.ns = "cube";
    gpoints.action = marker.action = visualization_msgs::Marker::ADD;
    gpoints.pose.orientation.w = marker.pose.orientation.w = 1.0;

    gpoints.type = visualization_msgs::Marker::POINTS;
    marker.type = visualization_msgs::Marker::CUBE;  

    // POINTS markers use x and y scale for width/height respectively
    gpoints.scale.x = 0.04;
    gpoints.scale.y = 0.04;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    for(unsigned int i=0; i<point_clusters.size(); ++i){

      gpoints.id = cg;
      marker.id = 0; 

      cg++;
      gpoints.color.g = rand() / double(RAND_MAX);
      gpoints.color.b = rand() / double(RAND_MAX);
      gpoints.color.r = rand() / double(RAND_MAX);

      marker.color.a = 1.0; 
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.lifetime = ros::Duration();

      gpoints.color.a = 0.0;
      //gpoints.lifetime = ros::Duration(0.08);
      for(unsigned int j=0; j<point_clusters[i].size(); ++j){
        geometry_msgs::Point p;
        p.x = point_clusters[i][j].first;
        p.y = point_clusters[i][j].second;
        p.z = 0;
        gpoints.points.push_back(p);
      }
      marker_array.markers.push_back(gpoints);
      gpoints.points.clear();
    }

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    pub_marker_array.publish(marker_array);
    pub_marker.publish(marker);
    ROS_INFO("pub");

    r.sleep();
  }
}
