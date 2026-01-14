#ifndef GENERATE_COMPOSITE_BEZIER_CURVE_DATA_H
#define GENERATE_COMPOSITE_BEZIER_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

/**
 * @brief Generates control points for a Composite Cubic Bezier Curve.
 * The data approximates a circle using 4 spliced cubic Bezier segments.
 *
 * Logic based on CTestView::ReadPoint from the original MFC code.
 *
 * @param points Output vector. Will contain 13 points (4 segments * 3 + 1).
 */
inline void GenerateCompositeBezierCurveData(std::vector<Eigen::Vector3d>& points) {
    points.clear();

    double m = 0.5523; // Magic number for circle approximation with Bezier
    double r = 2.0;    // Radius (Scaled down from 200 for OpenGL view)
    double h = 5.0;
    // Raw points from the original logic (Indices 0 to 11)
    // We construct them directly into the vector in the order required for splicing.
    // Order: P0 -> P1 -> P2 -> P3 (Seg1) -> P4 -> P5 -> P6 (Seg2) ...
    // Note: P3 is the end of Seg1 and start of Seg2, so we don't add it twice.

    // Segment 1: Quadrant 1 (Right to Top)
    // P[0]
    points.push_back(Eigen::Vector3d(r, 0.0, h));
    // P[1]
    points.push_back(Eigen::Vector3d(r, m * r, h));
    // P[2]
    points.push_back(Eigen::Vector3d(m * r, r, h));
    // P[3] (Join Point 1)
    points.push_back(Eigen::Vector3d(0.0, r, h));

    // Segment 2: Quadrant 2 (Top to Left)
    // P[4]
    points.push_back(Eigen::Vector3d(-m * r, r, h));
    // P[5]
    points.push_back(Eigen::Vector3d(-r, m * r, h));
    // P[6] (Join Point 2)
    points.push_back(Eigen::Vector3d(-r, 0.0, h));

    // Segment 3: Quadrant 3 (Left to Bottom)
    // P[7]
    points.push_back(Eigen::Vector3d(-r, -m * r, h));
    // P[8]
    points.push_back(Eigen::Vector3d(-m * r, -r, h));
    // P[9] (Join Point 3)
    points.push_back(Eigen::Vector3d(0.0, -r, h));

    // Segment 4: Quadrant 4 (Bottom to Right)
    // P[10]
    points.push_back(Eigen::Vector3d(m * r, -r, h));
    // P[11]
    points.push_back(Eigen::Vector3d(r, -m * r, h));
    // P[0] (Join Point 4 - Closing the loop)
    points.push_back(Eigen::Vector3d(r, 0.0, h));
}

#endif // GENERATE_COMPOSITE_BEZIER_CURVE_DATA_H
