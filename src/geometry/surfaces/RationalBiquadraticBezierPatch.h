#ifndef RATIONAL_BIQUADRATIC_BEZIER_PATCH_H
#define RATIONAL_BIQUADRATIC_BEZIER_PATCH_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Rational Biquadratic Bezier Surface Patch.
 *
 * Unlike the standard Cubic Bezier Patch (4x4), this patch uses a 3x3 control grid
 * and explicit weights for each control point.
 *
 * Capabilities:
 * - Can EXACTLY represent parts of quadric surfaces (spheres, cylinders, cones).
 * - Defined by 9 control points (P) and 9 weights (w).
 */
class RationalBiquadraticBezierPatch {
public:
    RationalBiquadraticBezierPatch() {
        // Initialize weights to 1.0 (standard polynomial patch)
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m_weights[i][j] = 1.0;
    }

    ~RationalBiquadraticBezierPatch() = default;

    /**
     * @brief Sets the 3x3 control points and weights.
     *
     * @param points 3x3 array of Vector3 control points.
     * @param weights 3x3 array of scalar weights.
     */
    void SetControlData(const Vector3 points[3][3], const double weights[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m_controlPoints[i][j] = points[i][j];
                m_weights[i][j] = weights[i][j];
            }
        }
    }

    /**
     * @brief Evaluates the surface at (u, v) using the Rational Bernstein definition.
     *
     * P(u,v) = ( Sum(Bi(u)*Bj(v)*Wij*Pij) ) / ( Sum(Bi(u)*Bj(v)*Wij) )
     *
     * @param u Parameter [0, 1]
     * @param v Parameter [0, 1]
     * @return 3D Point on surface
     */
    Vector3 Evaluate(double u, double v) const {
        // 1. Calculate Quadratic Bernstein Basis Functions
        // B0 = (1-t)^2, B1 = 2t(1-t), B2 = t^2
        double u_inv = 1.0 - u;
        double v_inv = 1.0 - v;

        double Bu[3] = { u_inv * u_inv,  2.0 * u * u_inv,  u * u };
        double Bv[3] = { v_inv * v_inv,  2.0 * v * v_inv,  v * v };

        Vector3 numerator = Vector3::Zero();
        double denominator = 0.0;

        // 2. Double Summation
        for (int i = 0; i < 3; ++i) {     // u-direction rows
            for (int j = 0; j < 3; ++j) { // v-direction cols
                // Combined weight: Basis_u * Basis_v * weight_ij
                double w_basis = Bu[i] * Bv[j] * m_weights[i][j];

                numerator += m_controlPoints[i][j] * w_basis;
                denominator += w_basis;
            }
        }

        // Avoid division by zero
        if (std::abs(denominator) < 1e-9) return Vector3::Zero();

        return numerator / denominator;
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
     * @brief Generates the control net lines (3x3 grid) for visualization.
     */
    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        lineVertices.clear();
        // Horizontal lines (u-direction)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 2; ++j) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i][j + 1]);
            }
        }
        // Vertical lines (v-direction)
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 2; ++i) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i + 1][j]);
            }
        }
    }

private:
    Vector3 m_controlPoints[3][3];
    double m_weights[3][3];
};

#endif // RATIONAL_BIQUADRATIC_BEZIER_PATCH_H
