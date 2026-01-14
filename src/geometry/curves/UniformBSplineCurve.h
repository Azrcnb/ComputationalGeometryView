#ifndef UNIFORM_BSPLINE_CURVE_H
#define UNIFORM_BSPLINE_CURVE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D point representation
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a general Uniform B-Spline Curve of arbitrary order in 3D space.
 *
 * This class implements the De Boor algorithm for stable and efficient evaluation
 * of B-Spline curves. It replaces the legacy MFC/recursive implementation.
 */
class UniformBSplineCurve {
public:
    /**
     * @brief Constructor.
     * @param order The order of the B-Spline (k).
     *              k=2 is Linear, k=3 is Quadratic, k=4 is Cubic. Default is 3.
     */
    explicit UniformBSplineCurve(int order = 3) : m_order(order) {}

    ~UniformBSplineCurve() = default;

    /**
     * @brief Sets the order of the curve (k).
     *        Degree p = k - 1.
     * @param k The order (must be >= 2).
     */
    void SetOrder(int k) {
        if (k < 2) return;
        m_order = k;
    }

    /**
     * @brief Sets the 3D control points for the curve.
     *
     * @param points A vector of 3D control points.
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        m_points = points;
    }

    /**
     * @brief Generates the discrete vertices of the B-Spline curve.
     *
     * This function automatically generates a Uniform Knot Vector based on the
     * number of control points and the order. It then iterates through the
     * VALID domain of the B-Spline to generate points.
     *
     * @param sampleResolution The step size for the parameter t.
     *                         Note: In Uniform B-Splines, t usually ranges from [k-1, n+1].
     * @param outVertices Output vector where generated 3D points are stored.
     */
    void GenerateCurve(double sampleResolution, std::vector<Vector3>& outVertices) const {
        outVertices.clear();
        if (m_points.empty() || m_points.size() < static_cast<size_t>(m_order)) {
            // Not enough points to form a B-Spline of this order
            return;
        }

        // 1. Generate Uniform Knot Vector
        // Number of knots m = n + k + 1 (where n is index of last control point)
        // Here m_points.size() is n + 1. So Total Knots = size + k.
        std::vector<double> knots = GenerateUniformKnots(m_points.size(), m_order);

        // 2. Determine the valid domain [u_{k-1}, u_{n+1}]
        // The curve is only defined where we have full support of basis functions.
        // For uniform knots 0, 1, 2... the domain is [order-1, count].
        double tStart = knots[m_order - 1];
        double tEnd = knots[m_points.size()];

        // 3. Iterate and evaluate using De Boor's algorithm
        outVertices.reserve(static_cast<size_t>((tEnd - tStart) / sampleResolution) + 2);

        for (double t = tStart; t <= tEnd; t += sampleResolution) {
            outVertices.push_back(EvaluateDeBoor(t, knots));
        }

        // Ensure the exact end point is included
        if (std::abs(outVertices.back().x() - EvaluateDeBoor(tEnd, knots).x()) > 1e-6) {
            outVertices.push_back(EvaluateDeBoor(tEnd, knots));
        }
    }

private:
    int m_order; // Order k (Degree p = k - 1)
    std::vector<Vector3> m_points;

    /**
     * @brief Generates a standard uniform knot vector (0, 1, 2, 3...).
     *
     * @param numPoints Number of control points.
     * @param k Order of the curve.
     * @return std::vector<double> The knot vector.
     */
    std::vector<double> GenerateUniformKnots(size_t numPoints, int k) const {
        std::vector<double> knots;
        size_t numKnots = numPoints + k;
        knots.reserve(numKnots);

        // Standard uniform knots: 0, 1, 2, ...
        for (size_t i = 0; i < numKnots; ++i) {
            knots.push_back(static_cast<double>(i));
        }
        return knots;
    }

    /**
     * @brief Evaluates the B-Spline at parameter t using De Boor's Algorithm.
     *
     * De Boor's algorithm is a RATIONALization of De Casteljau's algorithm.
     * It is numerically stable and efficient.
     *
     * @param t The parameter value.
     * @param knots The knot vector.
     * @return Vector3 The interpolated point.
     */
    Vector3 EvaluateDeBoor(double t, const std::vector<double>& knots) const {
        int k = m_order;
        int p = k - 1; // Degree

        // 1. Find index 'j' such that knots[j] <= t < knots[j+1]
        // Since our knots are sorted, we can use upper_bound.
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        // Handle edge case where t is exactly the max value
        if (t >= knots[knots.size() - k]) {
            j = static_cast<int>(knots.size()) - k - 1;
        }

        // 2. Initialize the temporary control points d[]
        // We need points P[j-p] ... P[j]
        std::vector<Vector3> d;
        d.reserve(k);
        for (int i = 0; i <= p; ++i) {
            d.push_back(m_points[j - p + i]);
        }

        // 3. The Algorithm loop (Triangle computation)
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                // Calculate alpha
                // alpha = (t - u_{j-p+i}) / (u_{j+1+i-r} - u_{j-p+i})
                // Map indices to our knot vector logic:
                int index_low = j - p + i;
                int index_high = j + 1 + i - r;

                double denominator = knots[index_high] - knots[index_low];
                double alpha = 0.0;

                if (denominator != 0.0) {
                    alpha = (t - knots[index_low]) / denominator;
                }

                // Interpolate
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p];
    }
};

#endif // UNIFORM_BSPLINE_CURVE_H
