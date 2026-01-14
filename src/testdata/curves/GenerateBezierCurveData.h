#ifndef BEZIER_CURVE_DATA_H
#define BEZIER_CURVE_DATA_H

#include <vector>      // Required for std::vector
#include <Eigen/Dense> // Required for Eigen::Vector3d

// @Brief Function to generate test points for Cubic Spline
// @param points: A reference to a std::vector of Eigen::Vector3d objects.
//           The vector is cleared and then populated with the test points.
//           The 'points' vector is modified in place.
inline void GenerateBezierCurveData(std::vector<Eigen::Vector3d>& points) {
    // Clear any existing data in the vector to ensure a clean slate
    points.clear();

    // Add a series of 3D points to define a test path for the spline.
    // These points were chosen to create a visually interesting curve.
    points.push_back(Eigen::Vector3d(0.0, 0.0, 0.0)); // Point 0: Origin
    points.push_back(Eigen::Vector3d(1.0, 2.0, 0.0)); // Point 1
    points.push_back(Eigen::Vector3d(3.0, 2.0, 1.0)); // Point 2
    points.push_back(Eigen::Vector3d(4.0, 0.0, 0.0)); // Point 3
    points.push_back(Eigen::Vector3d(5.0, 2.0, 0.0)); // Point 4: Final point

    for (auto& p : points) {
        p.z() += 5.0;
    }
}

#endif // CUBIC_SPLINE_DATA_H