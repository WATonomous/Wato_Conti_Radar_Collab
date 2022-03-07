#ifndef CLUSTER_H
#define CLUSTER_H

#include <iostream>
#include <tuple>
#include <vector>

#include "point.h"

namespace dbscan {

class Cluster {
public:
    struct BoundingBox {
        double x_center, y_center;
        double x_dist, y_dist;
        BoundingBox(double x, double y, double x_dist, double y_dist)
            : x_center(x), y_center(y), x_dist(x_dist), y_dist(y_dist) {} 
    };

    Cluster(std::vector<Point> const& new_points);

    BoundingBox ConstructBoundingBox();

    std::tuple<double, double> Centroid();

    double IntraClusterDistance();

    int id() const;

    double magnitude() const;

    double azimuth() const;

    std::vector<Point> points;

    static int current_id;

    friend std::ostream& operator<<(std::ostream& out, const Cluster& c);

private:
    int id_;
    double magnitude_;
    double azimuth_;
};

}

#endif
