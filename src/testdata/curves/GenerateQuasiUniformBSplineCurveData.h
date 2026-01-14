#ifndef GENERATE_QUASI_UNIFORM_BSPLINE_CURVE_DATA_H
#define GENERATE_QUASI_UNIFORM_BSPLINE_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Function to generate test points for Quasi-Uniform B-Spline Curve
// @param points: A reference to a std::vector of Eigen::Vector3d objects.
//                The vector is cleared and then populated with the test points.
inline void GenerateQuasiUniformBSplineCurveData(std::vector<Eigen::Vector3d>& points) {
    points.clear();

    // Designing a set of points that clearly demonstrates the "Clamped" property.
    // The curve should start EXACTLY at P0 and end EXACTLY at P6.

    // P0: Start (Left-Bottom)
    points.push_back(Eigen::Vector3d(-4.0, -2.0, 0.0));

    // P1: Sharp rise (Control point only, curve will approximate this)
    points.push_back(Eigen::Vector3d(-3.0, 3.0, 1.0));

    // P2: Loop Top
    points.push_back(Eigen::Vector3d(-1.0, 3.5, 2.0));

    // P3: Dip Middle
    points.push_back(Eigen::Vector3d(0.0, 0.0, 1.5));

    // P4: Loop Top Right
    points.push_back(Eigen::Vector3d(1.0, 3.5, 2.0));

    // P5: Sharp descent
    points.push_back(Eigen::Vector3d(3.0, 3.0, 1.0));

    // P6: End (Right-Bottom) - Curve must finish here
    points.push_back(Eigen::Vector3d(4.0, -2.0, 0.0));
}

#endif // GENERATE_QUASI_UNIFORM_BSPLINE_CURVE_DATA_H
