#ifndef GENERATE_NON_UNIFORM_BSPLINE_CURVE_DATA_H
#define GENERATE_NON_UNIFORM_BSPLINE_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Function to generate test points for Non-Uniform B-Spline (Chord Length Parameterized)
// @param points: A reference to a std::vector of Eigen::Vector3d objects.
//
// This dataset features unevenly spaced points to demonstrate the algorithm's capability.
// - Cluster of points (P0, P1, P2) very close to each other.
// - Large gap to P3.
// - Moderate gap to P4.
// - Large gap to P5.
inline void GenerateNonUniformBSplineCurveData(std::vector<Eigen::Vector3d>& points) {
    points.clear();

    // 1. Dense Cluster (Simulation of high-detail area)
    points.push_back(Eigen::Vector3d(-4.0, 0.0, 5.0));   // P0
    points.push_back(Eigen::Vector3d(-3.8, 0.5, 5.2));   // P1 (Very close)
    points.push_back(Eigen::Vector3d(-3.6, -0.2, 5.0));  // P2 (Very close)

    // 2. Long Jump (Simulation of low-detail/straight area)
    // Distance from P2 (~-3.6) to P3 (0.0) is large (~3.6 units) compared to previous 0.2 units.
    points.push_back(Eigen::Vector3d(0.0, 2.5, 6.0));   // P3

    // 3. Medium Segment
    points.push_back(Eigen::Vector3d(2.0, 0.0, 4.0));  // P4

    // 4. Another Long Jump
    points.push_back(Eigen::Vector3d(4.5, 3.0, 5.0));   // P5
}

#endif // GENERATE_NON_UNIFORM_BSPLINE_CURVE_DATA_H
