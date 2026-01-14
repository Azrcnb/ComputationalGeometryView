#ifndef GENERATE_RATIONAL_BEZIER_CURVE_DATA_H
#define GENERATE_RATIONAL_BEZIER_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

/**
 * @brief Generates test control points for Rational Quadratic Bezier Curve.
 *
 * Creates 3 points forming a "V" or corner shape to clearly show
 * how the weight pulls the curve.
 *
 * @param points Output vector. Will contain exactly 3 points.
 */
inline void GenerateRationalBezierCurveData(std::vector<Eigen::Vector3d>& points) {
    points.clear();

    // Define 3 control points on the XY plane
    // P0: Start
    points.push_back(Eigen::Vector3d(-2.0, 0.0, 0.0));
    // P1: Control Point (Apex)
    points.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
    // P2: End
    points.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
}

#endif // GENERATE_RATIONAL_BEZIER_CURVE_DATA_H
