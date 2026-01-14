#ifndef PIECEWISE_BEZIER_CURVE_H
#define PIECEWISE_BEZIER_CURVE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D point representation
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Piecewise Bezier Curve implemented as a B-Spline.
 *
 * This class mimics the behavior of stitching multiple Bezier curves together
 * by using a specific B-Spline knot vector configuration (internal knots with
 * multiplicity = degree).
 *
 * This allows for C0 continuity (sharp corners) at the join points if desired,
 * matching the behavior of the legacy code provided.
 */
class PiecewiseBezierCurve {
public:
    /**
     * @brief Constructor.
     * @param order The order of the curve (k).
     *              k=4 (Cubic) is the standard for Piecewise Bezier.
     *              Degree p = k - 1.
     */
    explicit PiecewiseBezierCurve(int order = 4) : m_order(order) {}

    ~PiecewiseBezierCurve() = default;

    /**
     * @brief Sets the order of the curve (k).
     * @param k The order (must be >= 2).
     */
    void SetOrder(int k) {
        if (k < 2) return;
        m_order = k;
    }

    /**
     * @brief Sets the 3D control points.
     *
     * Note: For a valid Piecewise Bezier of degree p, the number of control points
     * must satisfy: (Points.size() - 1) % p == 0.
     * E.g., for Cubic (p=3), you need 4, 7, 10, 13... points.
     *
     * @param points A vector of 3D control points.
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        m_points = points;
    }

    /**
     * @brief Generates the curve points.
     *
     * Automatically constructs the "Piecewise Bezier" style knot vector
     * (0,0,0,0, 0.5,0.5,0.5, ... 1,1,1,1) and evaluates using De Boor.
     *
     * @param sampleResolution Step size for parameter t [0, 1].
     * @param outVertices Output container.
     */
    void GenerateCurve(double sampleResolution, std::vector<Vector3>& outVertices) const {
        outVertices.clear();

        int p = m_order - 1; // Degree
        if (m_points.empty() || m_points.size() < static_cast<size_t>(m_order)) {
            return;
        }

        // Validate if points count supports full Bezier segments
        // If (N-1) is not divisible by Degree, the last segment will be malformed 
        // in a strict Piecewise Bezier sense, but B-Spline math still works.
        // We proceed but it's good to be aware.

        // 1. Generate the specific Multi-Knot vector
        std::vector<double> knots = GeneratePiecewiseKnots(m_points.size(), m_order);

        // 2. Define Domain [0, 1]
        // Because of the clamped knot vector (0...0 to 1...1), valid domain is [0, 1].
        double tStart = 0.0;
        double tEnd = 1.0;

        // 3. Evaluate
        size_t estimatedPoints = static_cast<size_t>((tEnd - tStart) / sampleResolution) + 2;
        outVertices.reserve(estimatedPoints);

        for (double t = tStart; t <= tEnd; t += sampleResolution) {
            outVertices.push_back(EvaluateDeBoor(t, knots));
        }

        // Ensure the exact endpoint
        if (outVertices.empty() || (outVertices.back() - m_points.back()).norm() > 1e-6) {
            outVertices.push_back(m_points.back());
        }
    }

private:
    int m_order; // Order k
    std::vector<Vector3> m_points;

    /**
     * @brief Generates the knot vector with high multiplicity for internal knots.
     *
     * Example for Cubic (k=4) with 2 segments (7 points):
     * Knots: [0, 0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1, 1]
     *
     * @param numPoints Total control points.
     * @param k Order.
     * @return The knot vector.
     */
    std::vector<double> GeneratePiecewiseKnots(size_t numPoints, int k) const {
        size_t numKnots = numPoints + k;
        std::vector<double> knots;
        knots.reserve(numKnots);

        int p = k - 1; // Degree

        // Calculate number of segments assuming proper point count
        // If points=7, k=4 -> (7-1)/3 = 2 segments.
        // We want to map the parameter t to [0, 1].
        // Internal knots needed: (NumSegments - 1).

        // However, a more robust way to build this vector for B-Splines representing 
        // piecewise Bezier is:
        // Start with k zeros.
        // Then for every segment boundary, add k-1 knots.
        // End with k ones.

        // 1. Start Knots (0.0)
        for (int i = 0; i < k; ++i) knots.push_back(0.0);

        // 2. Internal Knots
        // We determine how many segments we can form.
        // A segment consumes 'p' intervals of points effectively.
        // Logic: The "Piecewise Bezier" B-Spline vector looks like:
        // 0 (k times), 
        // 1/S (p times), 
        // 2/S (p times), ...
        // 1 (k times)

        int numSegments = (static_cast<int>(numPoints) - 1) / p;
        if (numSegments < 1) numSegments = 1;

        for (int i = 1; i < numSegments; ++i) {
            double val = static_cast<double>(i) / static_cast<double>(numSegments);
            // Multiplicity = Degree (p)
            for (int r = 0; r < p; ++r) {
                knots.push_back(val);
            }
        }

        // 3. End Knots (1.0)
        // We simply fill the remaining required knots with 1.0 to satisfy size = n + k
        while (knots.size() < numKnots) {
            knots.push_back(1.0);
        }

        return knots;
    }

    /**
     * @brief Standard De Boor Algorithm.
     *
     * @param t Parameter [0, 1].
     * @param knots Knot vector.
     * @return Point on curve.
     */
    Vector3 EvaluateDeBoor(double t, const std::vector<double>& knots) const {
        int k = m_order;
        int p = k - 1;

        // 1. Find index j
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        if (t >= knots[knots.size() - k]) {
            j = static_cast<int>(knots.size()) - k - 1;
        }

        // 2. Initialize d
        std::vector<Vector3> d;
        d.reserve(k);
        for (int i = 0; i <= p; ++i) {
            int idx = j - p + i;
            if (idx >= 0 && idx < static_cast<int>(m_points.size()))
                d.push_back(m_points[idx]);
            else
                d.push_back(Vector3::Zero());
        }

        // 3. Recursion
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                int index_low = j - p + i;
                int index_high = j + 1 + i - r;
                double denominator = knots[index_high] - knots[index_low];
                double alpha = 0.0;

                // Handling 0 denominator (frequent in this specific knot vector type)
                if (std::abs(denominator) > 1e-9) {
                    alpha = (t - knots[index_low]) / denominator;
                }
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }
        return d[p];
    }
};

#endif // PIECEWISE_BEZIER_CURVE_H
