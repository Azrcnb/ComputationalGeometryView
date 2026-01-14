#ifndef GENERATE_CUBIC_UNIFORM_BSPLINE_PATCH_DATA_H
#define GENERATE_CUBIC_UNIFORM_BSPLINE_PATCH_DATA_H

#include <Eigen/Dense>

// @Brief Generates a 4x4 grid of control points for a Bicubic B-Spline Patch.
// @param points: Output 4x4 array.
//
// The points form a wavy surface to demonstrate the smoothing capability of B-Splines.
inline void GenerateCubicUniformBSplinePatchData(Eigen::Vector3d points[4][4]) {
    // Grid spacing
    double spacing = 2.0;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double x = (i - 1.5) * spacing; // Centered around 0
            double z = (j - 1.5) * spacing; // Centered around 0

            // Set base height (Y)
            double y = 0.0;

            // Make the center points (1,1), (1,2), (2,1), (2,2) much higher
            // to create a "peak" or "dome" shape.
            if ((i == 1 || i == 2) && (j == 1 || j == 2)) {
                y = 4.0;
            }

            // Push corners down
            if ((i == 0 || i == 3) && (j == 0 || j == 3)) {
                y = -2.0;
            }

            points[i][j] = Eigen::Vector3d(x, y, z);
        }
    }
}

#endif // GENERATE_CUBIC_UNIFORM_BSPLINE_PATCH_DATA_H
