#ifndef GENERATE_NON_UNIFORM_BSPLINE_SURFACE_DATA_H
#define GENERATE_NON_UNIFORM_BSPLINE_SURFACE_DATA_H

#include <vector>
#include <Eigen/Dense>

// @Brief Generates control grid for Non-Uniform B-Spline Surface.
// @param grid: Output 2D vector [rows][cols] of Vector3.
// 
// Source Data Config:
// Rows (n+1) = 4
// Cols (m+1) = 5
// Scaled down by 0.01 for OpenGL Viewport.
inline void GenerateNonUniformBSplineSurfaceData(std::vector<std::vector<Eigen::Vector3d>>& grid) {
    grid.clear();

    // Resize grid to 4 rows
    grid.resize(4);

    // Scaling factor (Original data is large, e.g., 450. Scale to 4.5)
    double s = 0.01;

    // Row 0
    grid[0].push_back(Eigen::Vector3d(20 * s, 0 * s, 200 * s));
    grid[0].push_back(Eigen::Vector3d(100 * s, 100 * s, 150 * s));
    grid[0].push_back(Eigen::Vector3d(230 * s, 130 * s, 140 * s));
    grid[0].push_back(Eigen::Vector3d(350 * s, 100 * s, 150 * s));
    grid[0].push_back(Eigen::Vector3d(450 * s, 30 * s, 150 * s));

    // Row 1
    grid[1].push_back(Eigen::Vector3d(0 * s, 100 * s, 150 * s));
    grid[1].push_back(Eigen::Vector3d(30 * s, 100 * s, 120 * s));
    grid[1].push_back(Eigen::Vector3d(110 * s, 120 * s, 80 * s));
    grid[1].push_back(Eigen::Vector3d(200 * s, 150 * s, 50 * s));
    grid[1].push_back(Eigen::Vector3d(300 * s, 150 * s, 50 * s));

    // Row 2
    grid[2].push_back(Eigen::Vector3d(-130 * s, 100 * s, 50 * s));
    grid[2].push_back(Eigen::Vector3d(-40 * s, 120 * s, 50 * s));
    grid[2].push_back(Eigen::Vector3d(30 * s, 150 * s, 30 * s));
    grid[2].push_back(Eigen::Vector3d(50 * s, 150 * s, 0 * s));
    grid[2].push_back(Eigen::Vector3d(150 * s, 200 * s, 0 * s));

    // Row 3
    grid[3].push_back(Eigen::Vector3d(-250 * s, 50 * s, 0 * s));
    grid[3].push_back(Eigen::Vector3d(-150 * s, 100 * s, 0 * s));
    grid[3].push_back(Eigen::Vector3d(-100 * s, 150 * s, -50 * s));
    grid[3].push_back(Eigen::Vector3d(0 * s, 150 * s, -70 * s));
    grid[3].push_back(Eigen::Vector3d(80 * s, 220 * s, -70 * s));
}

#endif // GENERATE_NON_UNIFORM_BSPLINE_SURFACE_DATA_H
