#include <algorithm>
#include <stdexcept>

#include "evaluate.h"

namespace dbscan {

Evaluate::Evaluate() {}

double Evaluate::AverageIntraClusterDistance(const Dbscan& scan) {
    double total_distances = 0;
    for (auto cluster:scan.clusters) {
        total_distances += cluster.IntraClusterDistance();
    }
    return total_distances/scan.clusters.size();
}

double Evaluate::AverageInterClusterDistance(const Dbscan& scan) {
    double total_distances = 0;
    int total_comparisons = 0;
    for (int i = 0; i < scan.clusters.size(); i++) {
        for (int j = i + 1; j < scan.clusters.size(); j++) {
            for (auto start:scan.clusters[i].points) {
                for (auto end:scan.clusters[j].points) {
                    total_distances += start.EuclideanDistance(end);
                    total_comparisons++;
                }
            }
        }
    }
    double avg = total_distances/total_comparisons;
    return avg;
}

double Evaluate::SilhouetteScore(const Dbscan& scan) {
    if (scan.clusters.size() < 2) {
        throw std::runtime_error("Not enough clusters to generate score");
    }
    double a = AverageIntraClusterDistance(scan);
    double b = AverageInterClusterDistance(scan);

    return (b - a)/std::max(a, b);
}

std::tuple<double, double, int> Evaluate::TuneParams(const std::vector<Point>& points) {
    double optimal_epsilon = 0;
    int optimal_points = 0;
    double max_silhouette_score = -1;

    for (double i = 0.1; i < 100; i += 0.1) {
        for (int j = 1; j < points.size()/2; j++) {
            try {
                Dbscan new_scan(points, i, j);
                new_scan.CreateClusters();
                double score = SilhouetteScore(new_scan);
                std::cout << score << "\n";
                if (score > max_silhouette_score) {
                    max_silhouette_score = score;
                    optimal_epsilon = i;
                    optimal_points = j;
                }
            } catch (const std::exception& e) {
                continue;
            }
        } 
    }

    return {optimal_epsilon, max_silhouette_score, optimal_points};
}

}
