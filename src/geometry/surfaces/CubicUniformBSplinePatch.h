#ifndef CUBIC_UNIFORM_BSPLINE_PATCH_H
#define CUBIC_UNIFORM_BSPLINE_PATCH_H

#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Bicubic Uniform B-Spline Surface Patch.
 *
 * This patch is defined by a 4x4 grid of control points.
 * It uses the standard Uniform B-Spline basis matrix for evaluation.
 * Unlike Bezier patches, B-Spline patches do not typically interpolate their control points
 * (except at corners depending on knot multiplicity, but this is a standard single uniform patch).
 *
 * Formula: P(u,v) = U * M * P * M^T * V^T
 * Where M is the B-Spline basis matrix.
 */
class CubicUniformBSplinePatch {
public:
    CubicUniformBSplinePatch() {
        // Initialize Basis Matrix M_bs
        // 1/6 * [ -1  3 -3  1 ]
        //       [  3 -6  3  0 ]
        //       [ -3  0  3  0 ]
        //       [  1  4  1  0 ]
        m_basisMatrix << -1, 3, -3, 1,
            3, -6, 3, 0,
            -3, 0, 3, 0,
            1, 4, 1, 0;
        m_basisMatrix /= 6.0;
    }

    ~CubicUniformBSplinePatch() = default;

    /**
     * @brief Sets the 4x4 control points grid.
     *
     * @param points 4x4 array of Vector3 control points.
     */
    void SetControlPoints(const Vector3 points[4][4]) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m_controlPoints[i][j] = points[i][j];
            }
        }
    }

    /**
     * @brief Evaluates the surface at parameter (u, v).
     *
     * @param u Parameter [0, 1]
     * @param v Parameter [0, 1]
     * @return Vector3 Point on the surface
     */
    Vector3 Evaluate(double u, double v) const {
        // Construct parameter vectors U and V
        // U = [u^3, u^2, u, 1]
        Eigen::Vector4d U_vec(u * u * u, u * u, u, 1.0);
        Eigen::Vector4d V_vec(v * v * v, v * v, v, 1.0);

        // We need to compute: Result = U_vec^T * M * PointsMatrix * M^T * V_vec
        // Since PointsMatrix contains Vector3d, we calculate separately for X, Y, Z components.

        // 1. Precompute Left Weight Vector: W_u = U_vec^T * M
        Eigen::Vector4d Wu = U_vec.transpose() * m_basisMatrix;

        // 2. Precompute Right Weight Vector: W_v = M^T * V_vec (Equivalent to V^T * M transposed)
        // Or simpler: W_v = (V_vec^T * m_basisMatrix)^T
        Eigen::Vector4d Wv = V_vec.transpose() * m_basisMatrix;

        // 3. Summation
        Vector3 point = Vector3::Zero();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                // Point += Wu[i] * Wv[j] * P[i][j]
                point += Wu[i] * Wv[j] * m_controlPoints[i][j];
            }
        }

        return point;
    }

    /**
     * @brief Generates mesh data for OpenGL rendering.
     *
     * @param steps Tessellation level (e.g., 20).
     * @param vertices Output vector for vertices.
     * @param indices Output vector for triangle indices.
     */
    void GenerateSurface(int steps, std::vector<Vector3>& vertices, std::vector<unsigned int>& indices) {
        vertices.clear();
        indices.clear();

        if (steps < 1) steps = 1;
        double stepSize = 1.0 / static_cast<double>(steps);

        // 1. Generate Vertices
        // Note: For a single Uniform B-Spline patch, the valid domain is typically [0, 1] 
        // relative to the "segment" defined by the 4x4 window.
        for (int i = 0; i <= steps; ++i) {
            double u = i * stepSize;
            for (int j = 0; j <= steps; ++j) {
                double v = j * stepSize;
                vertices.push_back(Evaluate(u, v));
            }
        }

        // 2. Generate Indices (Quads -> 2 Triangles)
        int rowLength = steps + 1;
        for (int i = 0; i < steps; ++i) {
            for (int j = 0; j < steps; ++j) {
                unsigned int p0 = i * rowLength + j;
                unsigned int p1 = p0 + 1;
                unsigned int p2 = (i + 1) * rowLength + j;
                unsigned int p3 = p2 + 1;

                // Triangle 1
                indices.push_back(p0);
                indices.push_back(p2);
                indices.push_back(p1);

                // Triangle 2
                indices.push_back(p1);
                indices.push_back(p2);
                indices.push_back(p3);
            }
        }
    }

    /**
     * @brief Generates the control net lines (4x4 grid) for visualization.
     */
    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        lineVertices.clear();

        // Horizontal lines (u-direction)
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i][j + 1]);
            }
        }

        // Vertical lines (v-direction)
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 3; ++i) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i + 1][j]);
            }
        }
    }

private:
    Vector3 m_controlPoints[4][4]; // 4x4 Control Grid
    Eigen::Matrix4d m_basisMatrix; // B-Spline Coefficient Matrix
};

#endif // CUBIC_UNIFORM_BSPLINE_PATCH_H
