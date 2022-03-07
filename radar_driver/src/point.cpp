#include "point.h"

namespace dbscan {

Point::Point(double x, double y, double magnitude, double azimuth)
    : x_(x), y_(y), magnitude_(magnitude), azimuth_(azimuth), state(PointState::unclassified) {}

double Point::EuclideanDistance(Point const& p) const {
    return std::pow(std::pow(x() - p.x(), 2) + std::pow(y() - p.y(), 2), 0.5);
}

bool Point::IsClassified() const {
    return state == PointState::classified; 
}

double Point::x() const {
    return x_;
}

double Point::y() const {
    return y_;
}

double Point::magnitude() const {
    return magnitude_;
}

double Point::azimuth() const {
    return azimuth_;
}

}
