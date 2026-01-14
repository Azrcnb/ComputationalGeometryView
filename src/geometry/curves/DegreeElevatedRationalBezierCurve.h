#ifndef DEGREE_ELEVATED_RATIONAL_BEZIER_CURVE_H
#define DEGREE_ELEVATED_RATIONAL_BEZIER_CURVE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Rational Quadratic Bezier Curve that is numerically elevated to Degree 3.
 *
 * Degree Elevation is a technique to increase the flexibility of a curve without changing its shape.
 * This class takes a standard Rational Quadratic (3 points) and computes the equivalent
 * Rational Cubic (4 points) representation, then renders it.
 */
class DegreeElevatedRationalBezierCurve {
public:
    DegreeElevatedRationalBezierCurve() {
        // Default initialization weights
        m_weights[0] = 1.0;
        m_weights[1] = 1.0;
        m_weights[2] = 1.0;
    }

    ~DegreeElevatedRationalBezierCurve() = default;

    /**
     * @brief Sets the 3 control points for the original quadratic curve.
     *
     * @param points A vector containing exactly 3 points (P0, P1, P2).
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        if (points.size() < 3) return;
        m_points[0] = points[0];
        m_points[1] = points[1];
        m_points[2] = points[2];
        m_needsUpdate = true;
    }

    /**
     * @brief Sets the weight for the middle control point (P1).
     *
     * @param w1 The weight factor (e.g., sqrt(2)/2 for a circular arc).
     */
    void SetMiddleWeight(double w1) {
        if (std::abs(m_weights[1] - w1) > 1e-9) {
            m_weights[1] = w1;
            m_needsUpdate = true;
        }
    }

    /**
     * @brief Generates a discrete list of vertices representing the curve using the ELEVATED (Cubic) formula.
     *
     * 1. Performs degree elevation algorithm to get 4 control points (V) and 4 weights (W).
     * 2. Evaluates the resulting rational cubic curve.
     *
     * @param tStep The step size for sampling parameter t (e.g., 0.01).
     * @param outVertices Output vector where the generated 3D points will be stored.
     */
    void GenerateCurve(float tStep, std::vector<Vector3>& outVertices) {
        outVertices.clear();

        // 1. Perform Degree Elevation if parameters changed
        if (m_needsUpdate) {
            PerformDegreeElevation();
        }

        // Reserve memory
        size_t estimatedPoints = static_cast<size_t>(1.0f / tStep) + 2;
        outVertices.reserve(estimatedPoints);

        // 2. Sample the Elevated Curve (Rational Cubic)
        for (float t = 0.0f; t <= 1.0f; t += tStep) {
            outVertices.push_back(EvaluateCubic(t));
        }

        // Add exact end point
        if (!outVertices.empty()) {
            outVertices.push_back(m_elevatedPoints[3]);
        }
    }

private:
    // Original Quadratic Data
    Vector3 m_points[3];
    double m_weights[3];

    // Elevated Cubic Data
    Vector3 m_elevatedPoints[4]; // V[0]..V[3]
    double m_elevatedWeights[4]; // W[0]..W[3]

    bool m_needsUpdate = true;

    /**
     * @brief Implements the Degree Elevation formula from n=2 to n=3.
     *
     * Formula:
     * Alpha = i / (n+1)  => i / 3
     * W[i] = Alpha * w[i-1] + (1 - Alpha) * w[i]
     * V[i] = (Alpha * w[i-1] * P[i-1] + (1 - Alpha) * w[i] * P[i]) / W[i]
     *
     * Boundary conditions handled by extending loops implicitly:
     * i=0: Alpha=0. w[-1] doesn't exist, term vanishes. Result is w[0], P[0].
     * i=3: Alpha=1. w[3] doesn't exist, term vanishes. Result is w[2], P[2].
     */
    void PerformDegreeElevation() {
        int n = 2; // Original degree
        int newDegree = n + 1; // 3

        // To safely handle boundaries without out-of-bounds access, we can unroll 
        // or strictly follow the logic where w[-1] is 0. 
        // However, the standard formula maps indices clearly:

        // V0 = P0, W0 = w0
        m_elevatedWeights[0] = m_weights[0];
        m_elevatedPoints[0] = m_points[0];

        // V3 = P2, W3 = w2
        m_elevatedWeights[3] = m_weights[2];
        m_elevatedPoints[3] = m_points[2];

        // Calculate internal points (i=1, i=2)
        for (int i = 1; i < newDegree; ++i) {
            double alpha = static_cast<double>(i) / static_cast<double>(newDegree);

            // W[i] = alpha * w[i-1] + (1-alpha) * w[i]
            // Note: In the source loop `w` array indices map directly because
            // the source `P` has size n+1 (0..2). 
            // Loop runs i=1, i=2. 
            // i=1: uses w[0], w[1]
            // i=2: uses w[1], w[2]

            m_elevatedWeights[i] = alpha * m_weights[i - 1] + (1.0 - alpha) * m_weights[i];

            // Numerator for V[i]
            Vector3 num = alpha * m_weights[i - 1] * m_points[i - 1] +
                (1.0 - alpha) * m_weights[i] * m_points[i];

            m_elevatedPoints[i] = num / m_elevatedWeights[i];
        }

        m_needsUpdate = false;
    }

    /**
     * @brief Evaluates the Rational Cubic Bezier point at t.
     * Uses the elevated control points (V) and weights (W).
     */
    Vector3 EvaluateCubic(double t) const {
        double oneMinusT = 1.0 - t;
        double oneMinusT2 = oneMinusT * oneMinusT;
        double t2 = t * t;

        // Cubic Bernstein Basis
        double b0 = oneMinusT * oneMinusT2;      // (1-t)^3
        double b1 = 3.0 * t * oneMinusT2;        // 3t(1-t)^2
        double b2 = 3.0 * t2 * oneMinusT;        // 3t^2(1-t)
        double b3 = t * t2;                      // t^3

        // Weighted Basis
        double wb0 = b0 * m_elevatedWeights[0];
        double wb1 = b1 * m_elevatedWeights[1];
        double wb2 = b2 * m_elevatedWeights[2];
        double wb3 = b3 * m_elevatedWeights[3];

        double denominator = wb0 + wb1 + wb2 + wb3;

        if (std::abs(denominator) < 1e-9) return Vector3::Zero();

        Vector3 numerator = wb0 * m_elevatedPoints[0] +
            wb1 * m_elevatedPoints[1] +
            wb2 * m_elevatedPoints[2] +
            wb3 * m_elevatedPoints[3];

        return numerator / denominator;
    }
};

#endif // DEGREE_ELEVATED_RATIONAL_BEZIER_CURVE_H
