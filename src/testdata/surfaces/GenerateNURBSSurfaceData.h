#ifndef GENERATE_NURBS_SURFACE_DATA_H
#define GENERATE_NURBS_SURFACE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Generates Control Points and Weights for a NURBS Surface.
// @param points: Output grid [rows][cols] of Vector3.
// @param weights: Output grid [rows][cols] of doubles.
//
// This dataset demonstrates the "Weighted Pull":
// A 4x5 grid where the center point is pulled up geometrically,
// AND has a very high weight (10.0) to sharpen the peak.
inline void GenerateNURBSSurfaceData(std::vector<std::vector<Eigen::Vector3d>>& points,
    std::vector<std::vector<double>>& weights) {
    points.clear();
    weights.clear();

    int rows = 4;
    int cols = 5;

    points.resize(rows);
    weights.resize(rows);

    double spacing = 2.0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            double x = (i - 1.5) * spacing;
            double z = (j - 2.0) * spacing;
            double y = 0.0;
            double w = 1.0; // Default weight

            // Create a geometric "hill"
            if ((i == 1 || i == 2) && (j == 1 || j == 2 || j == 3)) {
                y = 2.0;
            }

            // --- The NURBS Effect ---
            // Make the absolute center point (i=1, j=2) very heavy
            // This will pull the surface TIGHTLY towards (0, 5.0, 0)
            if (i == 1 && j == 2) {
                y = 5.0;  // Geometric peak
                w = 10.0; // NURBS Weight Pull
            }

            points[i].push_back(Eigen::Vector3d(x, y, z));
            weights[i].push_back(w);
        }
    }
}

#endif // GENERATE_NURBS_SURFACE_DATA_H
