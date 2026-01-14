#ifndef GENERATE_RATIONAL_BEZIER_PATCH_2X1_DATA_H
#define GENERATE_RATIONAL_BEZIER_PATCH_2X1_DATA_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

using Vector3 = Eigen::Vector3d;

/**
 * @brief Generates data for a 90-degree Cylinder Surface Patch.
 *
 * Geometry:
 * - Radius: r
 * - Height: h (along Y axis)
 * - Arc: 90 degrees in XZ plane (from X-axis to Z-axis).
 *
 * Grid 3x2:
 * - U (0..2): Quadratic Arc
 * - V (0..1): Linear Extrusion
 */
inline void GenerateCylinderPatchData(Vector3(&outPoints)[3][2], double(&outWeights)[3][2]) {
    double r = 3.0;
    double h = 5.0;
    double w_arc = std::sqrt(2.0) / 2.0; // cos(45)

    // --- Row 0 (Bottom Arc, v=0, y=0) ---
    // P00: Start on X-axis
    outPoints[0][0] = Vector3(r, 0.0, 0.0);  outWeights[0][0] = 1.0;
    // P10: Corner (Control Point)
    outPoints[1][0] = Vector3(r, 0.0, r);    outWeights[1][0] = w_arc;
    // P20: End on Z-axis
    outPoints[2][0] = Vector3(0.0, 0.0, r);  outWeights[2][0] = 1.0;

    // --- Row 1 (Top Arc, v=1, y=h) ---
    // Just translated up by h
    // P01
    outPoints[0][1] = Vector3(r, h, 0.0);    outWeights[0][1] = 1.0;
    // P11
    outPoints[1][1] = Vector3(r, h, r);      outWeights[1][1] = w_arc;
    // P21
    outPoints[2][1] = Vector3(0.0, h, r);    outWeights[2][1] = 1.0;
}

/**
 * @brief Generates data for a 90-degree Cone Surface Patch.
 *
 * Geometry:
 * - Base Radius: r
 * - Height: h
 * - Apex: At (0, h, 0)
 *
 * Grid 3x2:
 * - Row 0 (Base): 90-degree arc.
 * - Row 1 (Top): All points collapse to the Apex.
 */
inline void GenerateConePatchData(Vector3(&outPoints)[3][2], double(&outWeights)[3][2]) {
    double r = 3.0;
    double h = 5.0;
    double w_arc = std::sqrt(2.0) / 2.0;

    // --- Row 0 (Base Arc, v=0, y=0) ---
    // Same as cylinder
    outPoints[0][0] = Vector3(r, 0.0, 0.0);  outWeights[0][0] = 1.0;
    outPoints[1][0] = Vector3(r, 0.0, r);    outWeights[1][0] = w_arc;
    outPoints[2][0] = Vector3(0.0, 0.0, r);  outWeights[2][0] = 1.0;

    // --- Row 1 (Apex, v=1, y=h) ---
    // All control points converge to the apex (0, h, 0)
    // Note: Weights here technically don't change geometric position since points are identical,
    // but usually kept consistent with the row below for rational stability.
    Vector3 apex(0.0, h, 0.0);

    outPoints[0][1] = apex;   outWeights[0][1] = 1.0;
    outPoints[1][1] = apex;   outWeights[1][1] = w_arc;
    outPoints[2][1] = apex;   outWeights[2][1] = 1.0;
}

#endif // GENERATE_RATIONAL_BEZIER_PATCH_2X1_DATA_H
