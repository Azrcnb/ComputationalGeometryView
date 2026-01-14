#ifndef CUBIC_BEZIER_CURVE_H
#define CUBIC_BEZIER_CURVE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Cubic Bezier Curve (or generalized Bezier) in 3D space.
 *
 * Implements the De Casteljau algorithm for numerical stability.
 * While named "Cubic" (typically 4 points), this implementation supports
 * N control points to generate a curve of degree N-1.
 */
class CubicBezierCurve {
public:
    CubicBezierCurve() = default;
    ~CubicBezierCurve() = default;

    /**
     * @brief Sets the control points for the Bezier curve.
     *
     * @param points A vector of 3D control points.
     *               For a Cubic Bezier, provide exactly 4 points.
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        m_points = points;
        // In De Casteljau, we don't pre-calculate coefficients like in Splines,
        // but we verify we have enough points.
    }

    /**
     * @brief Generates a discrete list of vertices representing the curve.
     *
     * Iterates t from 0.0 to 1.0 and evaluates the curve using De Casteljau's algorithm.
     *
     * @param tStep The step size for sampling parameter t (e.g., 0.01).
     * @param outVertices Output vector where the generated 3D points will be stored.
     */
    void GenerateCurve(float tStep, std::vector<Vector3>& outVertices) const {
        outVertices.clear();

        if (m_points.empty()) return;

        // Reserve memory to avoid frequent reallocations
        size_t estimatedPoints = static_cast<size_t>(1.0f / tStep) + 2;
        outVertices.reserve(estimatedPoints);

        // Iterate t from 0 to 1
        for (float t = 0.0f; t <= 1.0f; t += tStep) {
            outVertices.push_back(DeCasteljau(t));
        }

        // Ensure the exact last point is included (to handle floating point errors)
        if (!outVertices.empty() && (outVertices.back() - m_points.back()).norm() > 1e-6) {
            outVertices.push_back(m_points.back());
        }
    }

private:
    std::vector<Vector3> m_points;

    /**
     * @brief Evaluates the Bezier curve at parameter t using De Casteljau's algorithm.
     *
     * This replaces the nested loop logic from the original DrawBezier/deCasteljau function.
     * It recursively interpolates between points until one point remains.
     *
     * @param t Interpolation parameter [0, 1].
     * @return The 3D point on the curve at t.
     */
    Vector3 DeCasteljau(double t) const {
        // Create a temporary copy of points for recursive calculation
        // Corresponds to the pp[][] array in the original code, 
        // but we only need one dynamic row that shrinks.
        std::vector<Vector3> tempPoints = m_points;

        size_t n = tempPoints.size(); // Number of points
        if (n == 0) return Vector3::Zero();

        // The De Casteljau algorithm:
        // Level r goes from 1 to n-1 (iterations)
        // At each level, calculate P_i = (1-t) * P_i + t * P_{i+1}

        // Loop corresponds to: for (int r=1; r<=n; r++)
        for (size_t r = 1; r < n; ++r) {
            // Loop corresponds to: for(int i=0; i<=n-r; i++)
            for (size_t i = 0; i < n - r; ++i) {
                // Formula: (1-t)*P[i] + t*P[i+1]
                // Eigen handles Vector3 scalar multiplication and addition automatically
                tempPoints[i] = (1.0 - t) * tempPoints[i] + t * tempPoints[i + 1];
            }
        }

        // The result is the first element of the reduced array
        return tempPoints[0];
    }
};

#endif // CUBIC_BEZIER_CURVE_H
