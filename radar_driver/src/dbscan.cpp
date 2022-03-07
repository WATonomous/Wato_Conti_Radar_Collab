#include <dbscan.h>
#include <cluster.h>

#include <algorithm>
#include <stdexcept>

namespace dbscan {

Dbscan::Dbscan(const std::vector<Point>& points, const int min_points, const double epsilon)
    : points(points), clusters({}), min_points(min_points), epsilon(epsilon) {}

std::vector<int> Dbscan::RegionQuery(const int current_index) {
    std::vector<int> neighbours = {};
    for (int i = 0; i < points.size(); i++) {
        if (points[i].EuclideanDistance(points[current_index]) <= epsilon && i != current_index) {
            neighbours.push_back(i);
        }
    }
    return neighbours;
}

std::vector<int> Dbscan::ExpandCluster(const int current_index) {
    std::vector<int> neighbour_seeds = RegionQuery(current_index);

    if (neighbour_seeds.size() < min_points) {
        points[current_index].state = Point::PointState::unclassified;
        return {};
    } else {
        points[current_index].state = Point::PointState::classified;
        for (auto index:neighbour_seeds) {
            if (!points[index].IsClassified()) {
                points[index].state = Point::PointState::classified;
            }
        }

        int i = 0;
        while (i < neighbour_seeds.size()) {
            int seed = neighbour_seeds[i];
            std::vector<int> new_seeds = RegionQuery(seed);
            if (!points[seed].IsClassified()) {
                points[seed].state = Point::PointState::classified;
            }

            if (new_seeds.size() >= min_points) {
                for (auto index:new_seeds) {
                    if (points[index].state == Point::PointState::unclassified) {
                        neighbour_seeds.push_back(index);
                    }
                    if (!points[index].IsClassified()) {
                        points[index].state = Point::PointState::classified;
                    }
                }
            }

            i++;
        }
    }

    return neighbour_seeds;
}

void Dbscan::CreateClusters() {
    for (int i = 0; i < points.size(); i++) {
        if (points[i].state == Point::PointState::unclassified) {
            std::vector<int> cluster_seeds = ExpandCluster(i);

            if (cluster_seeds.size() > 0) {
                std::vector<Point> cluster_points = {};
                for (auto index:cluster_seeds) {
                    cluster_points.push_back(points[index]);
                }

                Cluster new_cluster(cluster_points);
                clusters.push_back(new_cluster);
            }
        }
    }
}

}