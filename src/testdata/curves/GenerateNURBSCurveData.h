#ifndef GENERATE_NURBS_CURVE_DATA_H
#define GENERATE_NURBS_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Generates control points and weights for a NURBS Curve.
// @param points: Output control points.
// @param weights: Output weights.
//
// This example demonstrates the "Pulling Effect" of weights.
// We create a standard arch shape (5 points), but assign a HIGH weight
// to the center point. This causes the curve to be pulled tightly 
// towards the center control point, deviating from a standard B-Spline.
inline void GenerateNURBSCurveData(std::vector<Eigen::Vector3d>& points,
    std::vector<double>& weights) {
    points.clear();
    weights.clear();

    // 1. Define Control Points (Symmetric Arch)
    points.push_back(Eigen::Vector3d(-4.0, -2.0, 0.0)); // P0: Start
    points.push_back(Eigen::Vector3d(-2.0, 2.0, 1.0)); // P1
    points.push_back(Eigen::Vector3d(0.0, 4.0, 0.0)); // P2: Apex (High Weight)
    points.push_back(Eigen::Vector3d(2.0, 2.0, -1.0)); // P3
    points.push_back(Eigen::Vector3d(4.0, -2.0, 0.0)); // P4: End

    // 2. Define Weights
    // Standard B-Spline would have all weights = 1.0.
    // Here we give P2 a weight of 5.0 to pull the curve towards it.
    weights.push_back(1.0);
    weights.push_back(1.0);
    weights.push_back(5.0); // <--- The "Magnet"
    weights.push_back(1.0);
    weights.push_back(1.0);
}

#endif // GENERATE_NURBS_CURVE_DATA_H
