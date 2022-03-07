#ifndef POINT_H
#define POINT_H

#include <cmath>

namespace dbscan {

class Point {
public:
    enum class PointState {
        unclassified,
        noise,
        classified
    };

    Point(double x, double y, double magnitude, double azimuth);

    double EuclideanDistance(Point const& p) const;

    bool IsClassified() const;

    double x() const;

    double y() const;

    double magnitude() const;

    double azimuth() const;

    PointState state;

private:
    double x_, y_;
    double magnitude_;
    double azimuth_;
};

}

#endif
