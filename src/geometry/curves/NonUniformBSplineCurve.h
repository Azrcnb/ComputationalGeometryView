#ifndef NON_UNIFORM_BSPLINE_CURVE_H
#define NON_UNIFORM_BSPLINE_CURVE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D point representation
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Non-Uniform B-Spline Curve (NUBS) in 3D space.
 *
 * Unlike Uniform B-Splines, the knot vector in this class is determined by the
 * geometry of the control polygon (specifically, the chord lengths).
 * This implementation uses Chord Length Parameterization (inspired by the
 * Hartley-Judd algorithm logic from the provided snippet) to minimize
 * wiggles/overshoot when control points are unevenly spaced.
 */
class NonUniformBSplineCurve {
public:
    /**
     * @brief Constructor.
     * @param order The order of the B-Spline (k).
     *              Degree p = k - 1.
     */
    explicit NonUniformBSplineCurve(int order = 3) : m_order(order) {}

    ~NonUniformBSplineCurve() = default;

    /**
     * @brief Sets the order of the curve (k).
     * @param k The order (must be >= 2).
     */
    void SetOrder(int k) {
        if (k < 2) return;
        m_order = k;
    }

    /**
     * @brief Sets the 3D control points for the curve.
     * @param points A vector of 3D control points.
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        m_points = points;
    }

    /**
     * @brief Generates the discrete vertices of the Non-Uniform B-Spline curve.
     *
     * It calculates a non-uniform knot vector based on chord lengths, then
     * evaluates the curve using De Boor's algorithm.
     *
     * @param sampleResolution The step size for the parameter t [0, 1].
     * @param outVertices Output vector where generated 3D points are stored.
     */
    void GenerateCurve(double sampleResolution, std::vector<Vector3>& outVertices) const {
        outVertices.clear();

        // We need at least 'order' points to define a single segment
        if (m_points.empty() || m_points.size() < static_cast<size_t>(m_order)) {
            return;
        }

        // 1. Generate Non-Uniform Knot Vector (Chord Length Parameterization)
        std::vector<double> knots = GenerateChordLengthKnots(m_points, m_order);

        // 2. Determine the valid domain
        // For Clamped knot vectors (standard for NUBS), domain is [0, 1].
        // Specifically [knots[k-1], knots[n+1]]
        double tStart = 0.0;
        double tEnd = 1.0;

        // 3. Iterate and evaluate using De Boor's algorithm
        size_t estimatedPoints = static_cast<size_t>((tEnd - tStart) / sampleResolution) + 2;
        outVertices.reserve(estimatedPoints);

        for (double t = tStart; t <= tEnd; t += sampleResolution) {
            outVertices.push_back(EvaluateDeBoor(t, knots));
        }

        // Ensure the exact end point is included
        Vector3 endPoint = EvaluateDeBoor(tEnd, knots);
        if (outVertices.empty() || (outVertices.back() - endPoint).norm() > 1e-6) {
            outVertices.push_back(endPoint);
        }
    }

private:
    int m_order; // Order k
    std::vector<Vector3> m_points;

    /**
     * @brief Generates knots based on Chord Length Parameterization.
     *
     * This corresponds to the logic in the provided snippet where distances (sqrt)
     * between points affect the knot values.
     *
     * Algorithm:
     * 1. Calculate total chord length of the control polygon.
     * 2. Normalize accumulated lengths to range [0, 1].
     * 3. Construct knots such that internal intervals match these geometric proportions.
     *    Ends are clamped (multiplicity k).
     *
     * @param points Control points.
     * @param k Order.
     * @return std::vector<double> The non-uniform knot vector.
     */
    std::vector<double> GenerateChordLengthKnots(const std::vector<Vector3>& points, int k) const {
        size_t numPoints = points.size();
        size_t numKnots = numPoints + k;
        std::vector<double> knots(numKnots);
        int p = k - 1; // Degree

        // 1. Calculate Chord Lengths and Total Length
        // params[i] will store the parameter value t associated with point P[i]
        std::vector<double> params(numPoints);
        params[0] = 0.0;

        double totalLength = 0.0;
        for (size_t i = 1; i < numPoints; ++i) {
            double dist = (points[i] - points[i - 1]).norm();
            totalLength += dist;
            params[i] = totalLength;
        }

        // Normalize parameters to [0, 1]
        if (totalLength > 0.0) {
            for (size_t i = 1; i < numPoints; ++i) {
                params[i] /= totalLength;
            }
        }
        else {
            // Handle case where all points are same (degenerate)
            std::fill(params.begin(), params.end(), 0.0);
            params.back() = 1.0;
        }

        // 2. Construct Knot Vector

        // A. Clamped Start: First k knots are 0.0
        for (int i = 0; i < k; ++i) knots[i] = 0.0;

        // B. Clamped End: Last k knots are 1.0
        for (size_t i = numPoints; i < numKnots; ++i) knots[i] = 1.0;

        // C. Internal Knots
        // Ideally, internal knots relate to the parameters of the points.
        // A simple and robust method for arbitrary order k is De Boor's averaging:
        // u_{j+k-1} = (1/p) * Sum(t_{i}) for i from j to j+p-1 (Moving average)
        // However, for consistency with the "Chord Length" concept directly:
        // We often map knots directly to the parameters if N is sufficient.

        // Here we use a direct mapping which is common in "Non-Uniform" definitions
        // for shape preservation:
        // We need to fill knots from index [k] to [numPoints - 1].
        // There are (numPoints - k) internal knots.
        // We map them based on the normalized parameters.

        // Note: The logic below approximates the Hartley-Judd distribution by 
        // ensuring knot intervals correspond to geometric intervals.
        for (size_t j = 1; j < numPoints - p; ++j) {
            // Internal knot 'j' roughly corresponds to parameter at point 'j'
            // But we must handle the index shift due to degree.
            // Using a simple sliding average for smoothness (De Boor's method):
            double sum = 0.0;
            for (int m = j; m < j + p; ++m) {
                if (m < static_cast<int>(params.size()))
                    sum += params[m];
            }
            knots[j + k - 1] = sum / p;
        }

        return knots;
    }

    /**
     * @brief Evaluates the B-Spline at parameter t using De Boor's Algorithm.
     *
     * @param t The parameter value.
     * @param knots The knot vector.
     * @return Vector3 The interpolated point.
     */
    Vector3 EvaluateDeBoor(double t, const std::vector<double>& knots) const {
        int k = m_order;
        int p = k - 1;

        // 1. Find index 'j' such that knots[j] <= t < knots[j+1]
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        if (t >= knots[knots.size() - k]) {
            j = static_cast<int>(knots.size()) - k - 1;
        }

        // 2. Initialize temporary control points
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

                // Critical for Non-Uniform: Denominators vary significantly.
                if (std::abs(denominator) > 1e-9) {
                    alpha = (t - knots[index_low]) / denominator;
                }

                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p];
    }
};

#endif // NON_UNIFORM_BSPLINE_CURVE_H
