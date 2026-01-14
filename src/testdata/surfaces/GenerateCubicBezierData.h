#ifndef CUBIC_BEZIER_PATCH_DATA_H
#define CUBIC_BEZIER_PATCH_DATA_H

#include "geometry/surfaces/CubicBezierPatch.h" // Include header defining Vector3 type

// @Brief Function to generate test control points for a Cubic Bezier Patch
// This function fills a 4x4 array of Vector3 control points with specific values
// to define a simple, demonstrative 3D surface patch.
// The generated patch has a flat base but is raised in the center, forming a dome-like shape,
// with one corner slightly lowered to add variation.
//
// @param controlPoints: A reference to a 4x4 C-style array of Vector3 objects.
//                  The array elements are populated with calculated coordinates.
inline void GenerateCubicBezierData(Vector3(&controlPoints)[4][4]) {
    // Iterate through the 4 rows (u-direction) of the control grid
    for (int i = 0; i < 4; ++i) {
        // Iterate through the 4 columns (v-direction) of the control grid
        for (int j = 0; j < 4; ++j) {
            // Calculate X and Z coordinates based on grid indices.
            // This spreads the control points out on the XZ plane, starting from (0,0).
            double x = i * 2.0;
            double z = j * 2.0;
            double y = 0.0; // Start with a base Y value of 0.

            // Create a raised, convex area in the middle of the patch (indices 1,2 for both u and v)
            if ((i == 1 || i == 2) && (j == 1 || j == 2)) {
                y = 3.0; // Elevate these central points significantly.
            }

            // Slightly lower the corner point at index (3,3) to introduce asymmetry.
            if (i == 3 && j == 3) {
                y = 1.0; // Lower this specific corner point.
            }

            // Assign the calculated 3D coordinates to the corresponding control point in the grid.
            controlPoints[i][j] = Vector3(x, y, z);
        }
    }
}

#endif // CUBIC_BEZIER_PATCH_DATA_H