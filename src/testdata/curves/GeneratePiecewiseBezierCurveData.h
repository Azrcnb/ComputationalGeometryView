#ifndef GENERATE_PIECEWISE_BEZIER_CURVE_DATA_H
#define GENERATE_PIECEWISE_BEZIER_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Function to generate test points for Piecewise Bezier Curve (B-Spline representation)
// @param points: A reference to a std::vector of Eigen::Vector3d objects.
//
// We generate 7 points to form exactly 2 Cubic Bezier segments.
// Segment 1: P0, P1, P2, P3
// Segment 2: P3, P4, P5, P6
inline void GeneratePiecewiseBezierCurveData(std::vector<Eigen::Vector3d>& points) {
    points.clear();

    // Segment 1: An arch in the XY plane
    points.push_back(Eigen::Vector3d(-4.0, 0.0, 0.0)); // P0
    points.push_back(Eigen::Vector3d(-3.0, 3.0, 0.0)); // P1
    points.push_back(Eigen::Vector3d(-1.0, 3.0, 0.0)); // P2
    points.push_back(Eigen::Vector3d(0.0, 0.0, 0.0)); // P3 (Junction Point)

    // Segment 2: An arch in the XZ plane (Perpendicular to Segment 1)
    // Note: The transition from P2->P3 to P3->P4 is NOT smooth (Sharp Corner).
    // This demonstrates the ability of Piecewise Bezier to handle C0 continuity.
    points.push_back(Eigen::Vector3d(1.0, 0.0, 3.0)); // P4
    points.push_back(Eigen::Vector3d(3.0, 0.0, 3.0)); // P5
    points.push_back(Eigen::Vector3d(4.0, 0.0, 0.0)); // P6
}

#endif // GENERATE_PIECEWISE_BEZIER_CURVE_DATA_H
