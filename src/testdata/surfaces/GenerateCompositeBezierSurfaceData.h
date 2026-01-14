#ifndef GENERATE_COMPOSITE_BEZIER_SURFACE_DATA_H
#define GENERATE_COMPOSITE_BEZIER_SURFACE_DATA_H

#include <vector>
#include <array>
#include <Eigen/Dense>

// Using 3D vectors of Eigen library
using Vector3 = Eigen::Vector3d;

/**
 * @brief Helper structure to hold raw patch indices (4x4)
 */
struct RawPatchIndices {
    int indices[4][4];
};

/**
 * @brief Generates control points and patch indices for a Sphere approximation.
 * Based on the "Bezier Surface Splicing" algorithm (8 octants).
 *
 * @param outPoints Output vector for global control points (62 points).
 * @param outPatches Output vector for patch definitions (8 patches).
 */
inline void GenerateCompositeBezierSurfaceData(
    std::vector<Vector3>& outPoints,
    std::vector<RawPatchIndices>& outPatches)
{
    outPoints.clear();
    outPatches.clear();

    // Resize points vector to hold all 62 points (indices 0 to 61)
    outPoints.resize(62);

    double r = 5.0; // Radius (Scaled for visualization)
    double m = 0.5523; // Magic constant for circular approximation

    // --- 1. ReadVertex Logic (Converted) ---

    // 1st Octant
    outPoints[0] = Vector3(0.0, r, 0.0);
    outPoints[1] = Vector3(0.0, r, m * r);
    outPoints[2] = Vector3(0.0, m * r, r);
    outPoints[3] = Vector3(0.0, 0.0, r);
    outPoints[4] = Vector3(m * m * r, r, m * r);
    outPoints[5] = Vector3(m * r, m * r, r);
    outPoints[6] = Vector3(m * r, 0.0, r);
    outPoints[7] = Vector3(m * r, r, m * m * r);
    outPoints[8] = Vector3(r, m * r, m * r);
    outPoints[9] = Vector3(r, 0.0, m * r);
    outPoints[10] = Vector3(m * r, r, 0.0);
    outPoints[11] = Vector3(r, m * r, 0.0);
    outPoints[12] = Vector3(r, 0.0, 0.0);

    // 2nd Octant
    outPoints[13] = Vector3(m * r, r, -m * m * r);
    outPoints[14] = Vector3(r, m * r, -m * r);
    outPoints[15] = Vector3(r, 0.0, -m * r);
    outPoints[16] = Vector3(m * m * r, r, -m * r);
    outPoints[17] = Vector3(m * r, m * r, -r);
    outPoints[18] = Vector3(m * r, 0.0, -r);
    outPoints[19] = Vector3(0.0, r, -m * r);
    outPoints[20] = Vector3(0.0, m * r, -r);
    outPoints[21] = Vector3(0.0, 0.0, -r);

    // 3rd Octant
    outPoints[22] = Vector3(-m * m * r, r, -m * r);
    outPoints[23] = Vector3(-m * r, m * r, -r);
    outPoints[24] = Vector3(-m * r, 0.0, -r);
    outPoints[25] = Vector3(-m * r, r, -m * m * r);
    outPoints[26] = Vector3(-r, m * r, -m * r);
    outPoints[27] = Vector3(-r, 0.0, -m * r);
    outPoints[28] = Vector3(-m * r, r, 0.0);
    outPoints[29] = Vector3(-r, m * r, 0.0);
    outPoints[30] = Vector3(-r, 0.0, 0.0);

    // 4th Octant
    outPoints[31] = Vector3(-m * r, r, m * m * r);
    outPoints[32] = Vector3(-r, m * r, m * r);
    outPoints[33] = Vector3(-r, 0.0, m * r);
    outPoints[34] = Vector3(-m * m * r, r, m * r);
    outPoints[35] = Vector3(-m * r, m * r, r);
    outPoints[36] = Vector3(-m * r, 0.0, r);

    // 5th Octant
    outPoints[37] = Vector3(0.0, -m * r, r);
    outPoints[38] = Vector3(0.0, -r, m * r);
    outPoints[39] = Vector3(m * r, -m * r, r);
    outPoints[40] = Vector3(m * m * r, -r, m * r);
    outPoints[41] = Vector3(r, -m * r, m * r);
    outPoints[42] = Vector3(m * r, -r, m * m * r);
    outPoints[43] = Vector3(r, -m * r, 0.0);
    outPoints[44] = Vector3(m * r, -r, 0.0);

    // 6th Octant
    outPoints[45] = Vector3(r, -m * r, -m * r);
    outPoints[46] = Vector3(m * r, -r, -m * m * r);
    outPoints[47] = Vector3(m * r, -m * r, -r);
    outPoints[48] = Vector3(m * m * r, -r, -m * r);
    outPoints[49] = Vector3(0.0, -m * r, -r);
    outPoints[50] = Vector3(0.0, -r, -m * r);

    // 7th Octant
    outPoints[51] = Vector3(-m * r, -m * r, -r);
    outPoints[52] = Vector3(-m * m * r, -r, -m * r);
    outPoints[53] = Vector3(-r, -m * r, -m * r);
    outPoints[54] = Vector3(-m * r, -r, -m * m * r);
    outPoints[55] = Vector3(-r, -m * r, 0.0);
    outPoints[56] = Vector3(-m * r, -r, 0.0);

    // 8th Octant
    outPoints[57] = Vector3(-r, -m * r, m * r);
    outPoints[58] = Vector3(-m * r, -r, m * m * r);
    outPoints[59] = Vector3(-m * r, -m * r, r);
    outPoints[60] = Vector3(-m * m * r, -r, m * r);
    outPoints[61] = Vector3(0.0, -r, 0.0);

    // --- 2. ReadPatch Logic (Converted) ---
    // Helper lambda to easily push 4x4 arrays
    auto addPatch = [&](const int grid[4][4]) {
        RawPatchIndices p;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                p.indices[i][j] = grid[i][j];
        outPatches.push_back(p);
        };

    // Patch 0 (1st Octant)
    int s0[4][4] = {
        {0, 1, 2, 3},
        {0, 4, 5, 6},
        {0, 7, 8, 9},
        {0, 10, 11, 12}
    };
    addPatch(s0);

    // Patch 1 (2nd Octant)
    int s1[4][4] = {
        {0, 10, 11, 12},
        {0, 13, 14, 15},
        {0, 16, 17, 18},
        {0, 19, 20, 21}
    };
    addPatch(s1);

    // Patch 2 (3rd Octant)
    int s2[4][4] = {
        {0, 19, 20, 21},
        {0, 22, 23, 24},
        {0, 25, 26, 27},
        {0, 28, 29, 30}
    };
    addPatch(s2);

    // Patch 3 (4th Octant)
    int s3[4][4] = {
        {0, 28, 29, 30},
        {0, 31, 32, 33},
        {0, 34, 35, 36},
        {0, 1, 2, 3}
    };
    addPatch(s3);

    // Patch 4 (5th Octant)
    int s4[4][4] = {
        {3, 37, 38, 61},
        {6, 39, 40, 61},
        {9, 41, 42, 61},
        {12, 43, 44, 61}
    };
    addPatch(s4);

    // Patch 5 (6th Octant)
    int s5[4][4] = {
        {12, 43, 44, 61},
        {15, 45, 46, 61},
        {18, 47, 48, 61},
        {21, 49, 50, 61}
    };
    addPatch(s5);

    // Patch 6 (7th Octant)
    int s6[4][4] = {
        {21, 49, 50, 61},
        {24, 51, 52, 61},
        {27, 53, 54, 61},
        {30, 55, 56, 61}
    };
    addPatch(s6);

    // Patch 7 (8th Octant)
    int s7[4][4] = {
        {30, 55, 56, 61},
        {33, 57, 58, 61},
        {36, 59, 60, 61},
        {3, 37, 38, 61}
    };
    addPatch(s7);
}

#endif // GENERATE_COMPOSITE_BEZIER_SURFACE_DATA_H
