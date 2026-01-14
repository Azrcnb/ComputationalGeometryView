#ifndef GENERATE_RATIONAL_BIQUADRATIC_BEZIER_PATCH_DATA_H
#define GENERATE_RATIONAL_BIQUADRATIC_BEZIER_PATCH_DATA_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

using Vector3 = Eigen::Vector3d;

/**
 * @brief Generates data for an EXACT 1/8th Sphere using Rational Biquadratic Bezier Patch.
 *
 * Logic:
 * Represents the octant x>=0, y>=0, z>=0.
 * Uses a degenerate patch where v=1 collapses to the pole (0,0,r).
 *
 * @param outPoints 3x3 Array of Control Points
 * @param outWeights 3x3 Array of Weights
 */
inline void GenerateRationalBiquadraticBezierPatchData(
    Vector3(&outPoints)[3][3],
    double(&outWeights)[3][3])
{
    double r = 4.0; // Radius
    double w_mid = std::sqrt(2.0) / 2.0; // 0.7071...

    // Row 0 (Equator, v=0): An arc in the XY plane
    // P00(r,0,0) -> P01(r,r,0) -> P02(0,r,0)
    outPoints[0][0] = Vector3(r, 0.0, 0.0); outWeights[0][0] = 1.0;
    outPoints[0][1] = Vector3(r, r, 0.0);   outWeights[0][1] = w_mid;
    outPoints[0][2] = Vector3(0.0, r, 0.0); outWeights[0][2] = 1.0;

    // Row 1 (Mid-Latitude, v=0.5): An intermediate arc
    // Z coordinate is lifted to r.
    // Weights are multiplied: w_mid * w_mid = 0.5 for the center point.
    outPoints[1][0] = Vector3(r, 0.0, r);   outWeights[1][0] = w_mid;
    outPoints[1][1] = Vector3(r, r, r);     outWeights[1][1] = 0.5; // w_mid * w_mid
    outPoints[1][2] = Vector3(0.0, r, r);   outWeights[1][2] = w_mid;

    // Row 2 (North Pole, v=1): Degenerate row
    // All points collapse to (0,0,r). This creates the triangular topology.
    outPoints[2][0] = Vector3(0.0, 0.0, r); outWeights[2][0] = 1.0;
    outPoints[2][1] = Vector3(0.0, 0.0, r); outWeights[2][1] = w_mid;
    outPoints[2][2] = Vector3(0.0, 0.0, r); outWeights[2][2] = 1.0;
}

#endif // GENERATE_RATIONAL_BIQUADRATIC_BEZIER_PATCH_DATA_H
