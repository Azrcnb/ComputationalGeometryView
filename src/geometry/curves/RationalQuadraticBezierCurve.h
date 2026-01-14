#ifndef RATIONAL_QUADRATIC_BEZIER_CURVE_H
#define RATIONAL_QUADRATIC_BEZIER_CURVE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Rational Quadratic Bezier Curve in 3D space.
 *
 * This curve is defined by 3 control points (P0, P1, P2) and 3 weights (w0, w1, w2).
 * It is capable of representing conic sections exactly (circles, ellipses, hyperbolas, parabolas).
 *
 * Standard form: w0 = w2 = 1.0.
 * Shape control:
 * - w1 < 1: Ellipse
 * - w1 = 1: Parabola (Standard Quadratic Bezier)
 * - w1 > 1: Hyperbola
 */
class RationalQuadraticBezierCurve {
public:
    RationalQuadraticBezierCurve() {
        // Default initialization
        m_weights[0] = 1.0;
        m_weights[1] = 1.0;
        m_weights[2] = 1.0;
    }

    ~RationalQuadraticBezierCurve() = default;

    /**
     * @brief Sets the 3 control points for the quadratic curve.
     *
     * @param points A vector containing exactly 3 points (P0, P1, P2).
     *               If size != 3, the data will be ignored or truncated.
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        if (points.size() < 3) return;
        m_points[0] = points[0];
        m_points[1] = points[1];
        m_points[2] = points[2];
    }

    /**
     * @brief Sets the weight for the middle control point (P1).
     *
     * @param w1 The weight factor.
     *           w0 and w2 are kept at standard 1.0.
     *           Example: 2.0 (Hyperbola-like), 0.707 (Circle arc), 1.0 (Parabola).
     */
    void SetMiddleWeight(double w1) {
        m_weights[1] = w1;
    }

    /**
     * @brief Generates a discrete list of vertices representing the curve.
     *
     * Iterates t from 0.0 to 1.0 using the rational Bernstein formula.
     *
     * @param tStep The step size for sampling parameter t (e.g., 0.01).
     * @param outVertices Output vector where the generated 3D points will be stored.
     */
    void GenerateCurve(float tStep, std::vector<Vector3>& outVertices) const {
        outVertices.clear();

        // Reserve memory roughly
        size_t estimatedPoints = static_cast<size_t>(1.0f / tStep) + 2;
        outVertices.reserve(estimatedPoints);

        // Iterate t from 0 to 1
        for (float t = 0.0f; t <= 1.0f; t += tStep) {
            outVertices.push_back(Evaluate(t));
        }

        // Ensure the exact last point is included (P2)
        // Note: For rational curves, P(1) = P2 assuming w2 != 0
        if (!outVertices.empty()) {
            // Explicitly add the last control point to close the gap perfectly
            outVertices.push_back(m_points[2]);
        }
    }

private:
    Vector3 m_points[3]; // Fixed size: Quadratic curve always has 3 points
    double m_weights[3]; // Weights for P0, P1, P2

    /**
     * @brief Evaluates the Rational Quadratic Bezier point at parameter t.
     *
     * Formula:
     * Numerator = (1-t)^2 * w0 * P0 + 2t(1-t) * w1 * P1 + t^2 * w2 * P2
     * Denominator = (1-t)^2 * w0 + 2t(1-t) * w1 + t^2 * w2
     * P(t) = Numerator / Denominator
     *
     * @param t Interpolation parameter [0, 1].
     * @return The 3D point.
     */
    Vector3 Evaluate(double t) const {
        // Pre-calculate Bernstein basis terms
        double oneMinusT = 1.0 - t;
        double tt = t * t;

        // Basis functions for degree 2
        double bern02 = oneMinusT * oneMinusT;   // (1-t)^2
        double bern12 = 2.0 * t * oneMinusT;     // 2t(1-t)
        double bern22 = tt;                      // t^2

        // Weighted Basis Functions
        double wb0 = bern02 * m_weights[0];
        double wb1 = bern12 * m_weights[1];
        double wb2 = bern22 * m_weights[2];

        // Calculate Denominator (Scalar sum of weighted basis)
        double denominator = wb0 + wb1 + wb2;

        // Prevent division by zero (though unlikely with standard weights)
        if (std::abs(denominator) < 1e-9) {
            return Vector3::Zero();
        }

        // Calculate Numerator (Vector sum of weighted points)
        Vector3 numerator = wb0 * m_points[0] +
            wb1 * m_points[1] +
            wb2 * m_points[2];

        return numerator / denominator;
    }
};

#endif // RATIONAL_QUADRATIC_BEZIER_CURVE_H
