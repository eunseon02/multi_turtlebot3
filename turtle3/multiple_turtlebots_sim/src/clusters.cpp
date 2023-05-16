#include "clusters.hpp"


Clusters::Clusters(unsigned long int id, const pointList& points){
    this->id = id;
    this->r = rand() / double(RAND_MAX);
    this->g = rand() / double(RAND_MAX);
    this->b = rand() / double(RAND_MAX);
}
