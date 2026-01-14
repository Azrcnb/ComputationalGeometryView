#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// using 3D vectors of Eigen library
using Vector3 = Eigen::Vector3d;

class CubicBezierPatch {
public:
    // Constructor
    CubicBezierPatch() {
        InitBasisMatrix();
        // Initialize control points to zero
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m_controlPoints[i][j] = Vector3(0.0, 0.0, 0.0);
            }
        }
    }

    // Destructor
    ~CubicBezierPatch() {}

    // Set the 4x4 control points for the Bezier patch
    void SetControlPoints(const Vector3 points[4][4]) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m_controlPoints[i][j] = points[i][j];
            }
        }
    }

    // Evaluate a point on the surface at parameters (u, v), where u, v are in [0, 1]
    Vector3 Evaluate(double u, double v) const {
        // Construct parameter vectors U and V
        // U = [u^3, u^2, u, 1]^T
        // V = [v^3, v^2, v, 1]^T
        Eigen::Vector4d U;
        U << std::pow(u, 3), std::pow(u, 2), u, 1.0;

        Eigen::Vector4d V;
        V << std::pow(v, 3), std::pow(v, 2), v, 1.0;

        // Prepare matrices for X, Y, Z components of the control points grid (G)
        Eigen::Matrix4d Gx, Gy, Gz;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                Gx(i, j) = m_controlPoints[i][j].x();
                Gy(i, j) = m_controlPoints[i][j].y();
                Gz(i, j) = m_controlPoints[i][j].z();
            }
        }

        // Calculation Formula: P(u,v) = U^T * M * G * M^T * V
        // We compute this for each component (x, y, z) separately.
        // Note: Eigen handles the matrix multiplication efficiently.

        double x = U.transpose() * m_basisMatrix * Gx * m_basisMatrix.transpose() * V;
        double y = U.transpose() * m_basisMatrix * Gy * m_basisMatrix.transpose() * V;
        double z = U.transpose() * m_basisMatrix * Gz * m_basisMatrix.transpose() * V;

        return Vector3(x, y, z);
    }

    // Calculate mesh data for OpenGL rendering
    // steps: The tessellation level (e.g., 20 creates a 20x20 grid)
    // vertices: Output vector to store the generated 3D points
    // indices: Output vector to store triangle indices for glDrawElements
    void GenerateSurface(int steps, std::vector<Vector3>& vertices, std::vector<unsigned int>& indices) {
        vertices.clear();
        indices.clear();

        // Avoid division by zero
        if (steps < 1) steps = 1;

        double stepSize = 1.0 / static_cast<double>(steps);

        // 1. Generate Vertices
        // Loop through u and v from 0.0 to 1.0
        for (int i = 0; i <= steps; ++i) {
            double u = i * stepSize;
            for (int j = 0; j <= steps; ++j) {
                double v = j * stepSize;
                vertices.push_back(Evaluate(u, v));
            }
        }

        // 2. Generate Indices (GL_TRIANGLES)
        // We create quads represented by two triangles
        // Grid structure:
        // P0 -- P1
        // |   /  |
        // P2 -- P3
        int rowLength = steps + 1; // Number of vertices in one row
        for (int i = 0; i < steps; ++i) {
            for (int j = 0; j < steps; ++j) {
                // Calculate indices for the current quad
                unsigned int p0 = i * rowLength + j;       // Top-left
                unsigned int p1 = p0 + 1;                  // Top-right
                unsigned int p2 = (i + 1) * rowLength + j; // Bottom-left
                unsigned int p3 = p2 + 1;                  // Bottom-right

                // First triangle (P0 -> P2 -> P1)
                indices.push_back(p0);
                indices.push_back(p2);
                indices.push_back(p1);

                // Second triangle (P1 -> P2 -> P3)
                indices.push_back(p1);
                indices.push_back(p2);
                indices.push_back(p3);
            }
        }
    }

    // Get control grid segments for debugging/visualization
    // Generates a list of points meant to be drawn as GL_LINES
    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        lineVertices.clear();

        // Add horizontal lines (along v direction)
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i][j + 1]);
            }
        }

        // Add vertical lines (along u direction)
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 3; ++i) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i + 1][j]);
            }
        }
    }

private:
    Vector3 m_controlPoints[4][4]; // 4x4 Control Points
    Eigen::Matrix4d m_basisMatrix; // Bezier Basis Matrix (M)

    // Helper to initialize the basis matrix
    void InitBasisMatrix() {
        // Initialize the standard cubic Bezier basis matrix
        // [ -1  3 -3  1 ]
        // [  3 -6  3  0 ]
        // [ -3  3  0  0 ]
        // [  1  0  0  0 ]
        m_basisMatrix << 
            -1, 3, -3, 1,
            3, -6, 3, 0,
            -3, 3, 0, 0,
            1, 0, 0, 0;
    }
};
