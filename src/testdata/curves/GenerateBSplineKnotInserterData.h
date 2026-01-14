#ifndef GENERATE_BSPLINE_KNOT_INSERTER_DATA_H
#define GENERATE_BSPLINE_KNOT_INSERTER_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Generates initial data for Knot Insertion test.
// @param points: Output control points.
// @param knots: Output knot vector.
//
// We setup a simple Cubic B-Spline (Order k=4).
// 5 Control Points (P0...P4).
// Standard Clamped Knot Vector: [0,0,0,0, 0.5, 1,1,1,1]
inline void GenerateBSplineKnotInserterData(std::vector<Eigen::Vector3d>& points,
    std::vector<double>& knots) {
    points.clear();
    knots.clear();

    // 1. Define Control Points (A simple "M" or arch shape)
    points.push_back(Eigen::Vector3d(-4.0, -2.0, 0.0)); // P0
    points.push_back(Eigen::Vector3d(-2.0, 3.0, 0.0)); // P1
    points.push_back(Eigen::Vector3d(0.0, 0.5, 0.0)); // P2
    points.push_back(Eigen::Vector3d(2.0, 3.0, 0.0)); // P3
    points.push_back(Eigen::Vector3d(4.0, -2.0, 0.0)); // P4

    // 2. Define Knot Vector for Cubic (k=4)
    // Number of Knots = NumPoints + Order = 5 + 4 = 9.
    // We use a Clamped vector with one internal knot at 0.5.

    // Multiplicity 4 at start
    knots.push_back(0.0); knots.push_back(0.0); knots.push_back(0.0); knots.push_back(0.0);

    // Internal knot
    knots.push_back(0.5);

    // Multiplicity 4 at end
    knots.push_back(1.0); knots.push_back(1.0); knots.push_back(1.0); knots.push_back(1.0);
}

#endif // GENERATE_BSPLINE_KNOT_INSERTER_DATA_H
