#ifndef GENERATE_UNIFORM_BSPLINE_CURVE_DATA_H
#define GENERATE_UNIFORM_BSPLINE_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Function to generate test points for Uniform B-Spline Curve
// @param points: A reference to a std::vector of Eigen::Vector3d objects.
//                The vector is cleared and then populated with the test points.
inline void GenerateUniformBSplineCurveData(std::vector<Eigen::Vector3d>& points) {
    points.clear();

    // Based on the original MFC example but scaled down for OpenGL view
    // and extended with Z-coordinates for 3D visualization.
    // Original X range ~[-400, 400] scaled to ~[-4.0, 4.0]

    // P0: Start lower left
    points.push_back(Eigen::Vector3d(-4.6, -0.5, 0.0));

    // P1: High peak, pulled forward in Z
    points.push_back(Eigen::Vector3d(-3.5, 2.0, 2.0));

    // P2: Dip down, back to Z=0
    points.push_back(Eigen::Vector3d(-0.6, 2.4, 0.0));

    // P3: Low valley, pushed back in Z
    points.push_back(Eigen::Vector3d(0.6, -1.2, -2.0));

    // P4: Rise again
    points.push_back(Eigen::Vector3d(2.6, -1.0, 0.0));

    // P5: End high right, forward in Z
    points.push_back(Eigen::Vector3d(4.0, 2.1, 1.5));

    // Optional: Add more points to show higher order smoothing
    points.push_back(Eigen::Vector3d(5.5, 0.0, 0.0));
}

#endif // GENERATE_UNIFORM_BSPLINE_CURVE_DATA_H
