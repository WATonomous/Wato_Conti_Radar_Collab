#include <cluster.h>

#include <algorithm>

namespace dbscan {

Cluster::Cluster(std::vector<Point> const& new_points) {
    id_ = current_id;
    points = new_points;

    double total_magnitude = 0;
    double total_azimuth = 0;
    for (auto point:new_points) {
        total_magnitude += point.magnitude();
        total_azimuth += point.azimuth();
    }
    int n = new_points.size();
    magnitude_ = total_magnitude/n;
    azimuth_ = total_azimuth/n;

    current_id++;
}

Cluster::BoundingBox Cluster::ConstructBoundingBox() {
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    for (auto point:points) {
        x_min = std::min(x_min, point.x());
        y_min = std::min(y_min, point.y());
        x_max = std::max(x_max, point.x());
        y_max = std::max(y_max, point.y());
    }

    double x_center, y_center;
    std::tie(x_center, y_center) = Centroid();

    double x_dist = x_max - x_min;
    double y_dist = y_max - y_min;

    Cluster::BoundingBox new_box(x_center, y_center, x_dist, y_dist);
    return new_box;
}

std::tuple<double, double> Cluster::Centroid() {
    double x, y, z = 0;
    for (auto point:points) {
        x += point.x();
        y += point.y();
        ROS_ERROR("X: %f", point.x());
    }
    int n = points.size();
    return std::make_tuple(x/n, y/n);
}

double Cluster::IntraClusterDistance() {
    double total_distances = 0;
    int total_comparisons = 0;
    for (int i = 0; i < points.size(); i++) {
        for (int j = i + 1; j < points.size(); j++) {
            total_distances += points[i].EuclideanDistance(points[j]);
            total_comparisons++;
        }
    }
    double avg = total_distances/total_comparisons;
    return avg;
}

int Cluster::id() const {
    return id_;
}

double Cluster::magnitude() const {
    return magnitude_;
}

double Cluster::azimuth() const {
    return azimuth_;
}

int Cluster::current_id = 0;

std::ostream& operator<<(std::ostream& out, const Cluster& c) {
    for (auto point:c.points) {
        out << point.x() << "," << point.y() << "," << c.id() << "\n";
    }
    return out;
}

}
