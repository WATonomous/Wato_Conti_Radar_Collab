#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <tuple>

#include "point.h"
#include "cluster.h"

namespace dbscan {

class Dbscan {
public:
    Dbscan(const std::vector<Point>& points, const int min_points, const double epsilon);

    std::vector<int> RegionQuery(const int current_index);

    std::vector<int> ExpandCluster(const int current_index);

    void CreateClusters();

    std::vector<Point> points;

    std::vector<Cluster> clusters;

    const int min_points;

    const double epsilon;
};

}

#endif
