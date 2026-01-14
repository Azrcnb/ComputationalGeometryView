#ifndef GENERATE_DEGREE_ELEVATED_RATIONAL_BEZIER_CURVE_DATA_H
#define GENERATE_DEGREE_ELEVATED_RATIONAL_BEZIER_CURVE_DATA_H

#include <vector>
#include <Eigen/Dense>

/**
 * @brief Represents a single quadratic segment for the circle approximation.
 */
struct RationalSegmentData {
    Eigen::Vector3d P0, P1, P2; // 3 Control Points
    double w1;                  // Weight for P1 (P0, P2 have w=1.0)
};

/**
 * @brief Generates data for a full circle composed of 4 Rational Quadratic Bezier segments.
 *
 * Each segment covers 90 degrees.
 * To represent a perfect circular arc of 90 degrees:
 * - P0, P2 are on the circle.
 * - P1 is the intersection of tangents at P0 and P2 (the corner).
 * - w1 = cos(90/2) = cos(45) = sqrt(2)/2.
 *
 * @param outSegments Output vector containing 4 segments.
 */
inline void GenerateDegreeElevatedRationalBezierCurveData(std::vector<RationalSegmentData>& outSegments) {
    outSegments.clear();

    double r = 4.0; // Radius
    double w = std::sqrt(2.0) / 2.0; // 0.7071...

    // Segment 1: Quadrant 1 (0 to 90 deg)
    // Start: (r, 0), End: (0, r), Corner: (r, r)
    outSegments.push_back({
        Eigen::Vector3d(r, 0.0, 0.0),
        Eigen::Vector3d(r, r, 0.0),
        Eigen::Vector3d(0.0, r, 0.0),
        w
        });

    // Segment 2: Quadrant 2 (90 to 180 deg)
    // Start: (0, r), End: (-r, 0), Corner: (-r, r)
    outSegments.push_back({
        Eigen::Vector3d(0.0, r, 0.0),
        Eigen::Vector3d(-r, r, 0.0),
        Eigen::Vector3d(-r, 0.0, 0.0),
        w
        });

    // Segment 3: Quadrant 3 (180 to 270 deg)
    // Start: (-r, 0), End: (0, -r), Corner: (-r, -r)
    outSegments.push_back({
        Eigen::Vector3d(-r, 0.0, 0.0),
        Eigen::Vector3d(-r, -r, 0.0),
        Eigen::Vector3d(0.0, -r, 0.0),
        w
        });

    // Segment 4: Quadrant 4 (270 to 360 deg)
    // Start: (0, -r), End: (r, 0), Corner: (r, -r)
    outSegments.push_back({
        Eigen::Vector3d(0.0, -r, 0.0),
        Eigen::Vector3d(r, -r, 0.0),
        Eigen::Vector3d(r, 0.0, 0.0),
        w
        });
}

#endif // GENERATE_DEGREE_ELEVATED_RATIONAL_BEZIER_CURVE_DATA_H
