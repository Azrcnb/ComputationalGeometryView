#ifndef RATIONAL_BEZIER_PATCH_2X1_H
#define RATIONAL_BEZIER_PATCH_2X1_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Rational Bezier Surface Patch with degrees 2 (u) and 1 (v).
 *
 * Grid Size: 3x2 (3 rows in u, 2 columns in v).
 * Formula: Uses rational Bernstein polynomials.
 * - u direction: Quadratic (Bernstein degree 2)
 * - v direction: Linear (Bernstein degree 1)
 */
class RationalBezierPatch2x1 {
public:
    RationalBezierPatch2x1() {
        // Initialize weights to 1.0
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 2; ++j)
                m_weights[i][j] = 1.0;
    }

    ~RationalBezierPatch2x1() = default;

    /**
     * @brief Sets the 3x2 control points and weights.
     *
     * @param points 3x2 array of Vector3 control points.
     * @param weights 3x2 array of scalar weights.
     */
    void SetControlData(const Vector3 points[3][2], const double weights[3][2]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 2; ++j) {
                m_controlPoints[i][j] = points[i][j];
                m_weights[i][j] = weights[i][j];
            }
        }
    }

    /**
     * @brief Evaluates the surface at (u, v).
     *
     * Formula derived from the provided MFC code snippet:
     * Numerator = Sum( B_i^2(u) * B_j^1(v) * w_ij * P_ij )
     * Denominator = Sum( B_i^2(u) * B_j^1(v) * w_ij )
     *
     * @param u Parameter [0, 1] (Quadratic direction)
     * @param v Parameter [0, 1] (Linear direction)
     * @return 3D Point
     */
    Vector3 Evaluate(double u, double v) const {
        // 1. Basis Functions

        // Quadratic Basis for U (Degree 2)
        // B0 = (1-u)^2, B1 = 2u(1-u), B2 = u^2
        double u_inv = 1.0 - u;
        double Bu[3] = { u_inv * u_inv, 2.0 * u * u_inv, u * u };

        // Linear Basis for V (Degree 1)
        // B0 = (1-v), B1 = v
        double v_inv = 1.0 - v;
        double Bv[2] = { v_inv, v };

        Vector3 numerator = Vector3::Zero();
        double denominator = 0.0;

        // 2. Summation over 3x2 Grid
        for (int i = 0; i < 3; ++i) {     // u loops 0..2
            for (int j = 0; j < 2; ++j) { // v loops 0..1

                double w_basis = Bu[i] * Bv[j] * m_weights[i][j];

                numerator += m_controlPoints[i][j] * w_basis;
                denominator += w_basis;
            }
        }

        if (std::abs(denominator) < 1e-9) return Vector3::Zero();

        return numerator / denominator;
    }

    /**
     * @brief Generates mesh data for OpenGL rendering.
     *
     * @param steps Tessellation level.
     * @param vertices Output vertices.
     * @param indices Output triangle indices.
     */
    void GenerateSurface(int steps, std::vector<Vector3>& vertices, std::vector<unsigned int>& indices) {
        vertices.clear();
        indices.clear();

        if (steps < 1) steps = 1;
        double stepSize = 1.0 / static_cast<double>(steps);

        // Generate Vertices
        for (int i = 0; i <= steps; ++i) {
            double u = i * stepSize;
            for (int j = 0; j <= steps; ++j) {
                double v = j * stepSize;
                vertices.push_back(Evaluate(u, v));
            }
        }

        // Generate Indices (Quads -> Triangles)
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
     * @brief Generates control grid lines (3x2 grid).
     */
    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        lineVertices.clear();

        // U-direction lines (3 rows of length 2)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 1; ++j) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i][j + 1]);
            }
        }

        // V-direction lines (2 columns of length 3)
        for (int j = 0; j < 2; ++j) {
            for (int i = 0; i < 2; ++i) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i + 1][j]);
            }
        }
    }

private:
    Vector3 m_controlPoints[3][2]; // 3x2 Grid
    double m_weights[3][2];
};

#endif // RATIONAL_BEZIER_PATCH_2X1_H
