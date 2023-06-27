#include "clusters.hpp"


Clusters::Clusters(unsigned long int id, const pointList& points){
    this->id = id;
    this->r = rand() / double(RAND_MAX);
    this->g = rand() / double(RAND_MAX);
    this->b = rand() / double(RAND_MAX);
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
void Clusters::update(const pointList& new_points) {
    this->calcMean(new_points);
}