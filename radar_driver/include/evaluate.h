#ifndef EVALUATE_H
#define EVALUATE_H

#include <tuple>
#include <vector>

#include "dbscan.h"
#include "point.h"

namespace dbscan {

class Evaluate {
public:
    Evaluate();

    double AverageIntraClusterDistance(const Dbscan& scan);

    double AverageInterClusterDistance(const Dbscan& scan);

    double SilhouetteScore(const Dbscan& scan);

    std::tuple<double, double, int> TuneParams(const std::vector<Point>& points);
};

}

#endif
