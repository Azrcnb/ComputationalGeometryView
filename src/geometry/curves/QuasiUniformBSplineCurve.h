#ifndef QUASI_UNIFORM_BSPLINE_CURVE_H
#define QUASI_UNIFORM_BSPLINE_CURVE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D point representation
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Quasi-Uniform B-Spline Curve in 3D space.
 *
 * A Quasi-Uniform B-Spline is characterized by a knot vector where the
 * knots at the ends have multiplicity equal to the order 'k' of the curve.
 * This forces the curve to interpolate (pass through) the first and last
 * control points.
 */
class QuasiUniformBSplineCurve {
public:
    /**
     * @brief Constructor.
     * @param order The order of the B-Spline (k).
     *              Degree p = k - 1.
     *              k=3 is Quadratic, k=4 is Cubic.
     */
    explicit QuasiUniformBSplineCurve(int order = 3) : m_order(order) {}

    ~QuasiUniformBSplineCurve() = default;

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
     * @brief Generates the discrete vertices of the Quasi-Uniform B-Spline curve.
     *
     * Automatically generates a Quasi-Uniform Knot Vector (multiplicity k at ends).
     * The parameter t ranges from [0, 1].
     *
     * @param sampleResolution The step size for the parameter t (e.g., 0.01).
     * @param outVertices Output vector where generated 3D points are stored.
     */
    void GenerateCurve(double sampleResolution, std::vector<Vector3>& outVertices) const {
        outVertices.clear();
        if (m_points.empty() || m_points.size() < static_cast<size_t>(m_order)) {
            // Not enough points to form a B-Spline of this order
            return;
        }

        // 1. Generate Quasi-Uniform Knot Vector
        // Total knots = numPoints + order
        std::vector<double> knots = GenerateQuasiUniformKnots(m_points.size(), m_order);

        // 2. Determine the valid domain
        // For Quasi-Uniform knots normalized to [0, 1] with multiplicity k at ends:
        // Valid domain is exactly [0.0, 1.0].
        // Specifically: [ knots[k-1], knots[n+1] ] where knots[k-1]=0 and knots[n+1]=1
        double tStart = 0.0;
        double tEnd = 1.0;

        // 3. Iterate and evaluate using De Boor's algorithm
        size_t estimatedPoints = static_cast<size_t>((tEnd - tStart) / sampleResolution) + 2;
        outVertices.reserve(estimatedPoints);

        for (double t = tStart; t <= tEnd; t += sampleResolution) {
            outVertices.push_back(EvaluateDeBoor(t, knots));
        }

        // Ensure the exact end point (P_n) is included
        // In Quasi-Uniform splines, the curve MUST end exactly at the last control point.
        if (outVertices.empty() || (outVertices.back() - m_points.back()).norm() > 1e-6) {
            outVertices.push_back(m_points.back());
        }
    }

private:
    int m_order; // Order k
    std::vector<Vector3> m_points;

    /**
     * @brief Generates a Quasi-Uniform knot vector.
     *
     * Structure:
     * - First k knots are 0.0
     * - Last k knots are 1.0
     * - Middle knots are uniformly spaced between 0.0 and 1.0
     *
     * @param numPoints Number of control points (n+1).
     * @param k Order of the curve.
     * @return std::vector<double> The knot vector.
     */
    std::vector<double> GenerateQuasiUniformKnots(size_t numPoints, int k) const {
        size_t numKnots = numPoints + k;
        std::vector<double> knots(numKnots);

        // 1. First k knots = 0.0
        for (int i = 0; i < k; ++i) {
            knots[i] = 0.0;
        }

        // 2. Last k knots = 1.0
        // (Indices from numPoints to numPoints + k - 1)
        for (size_t i = numPoints; i < numKnots; ++i) {
            knots[i] = 1.0;
        }

        // 3. Middle knots uniformly spaced
        // We need to fill indices from [k] to [numPoints - 1].
        // Number of internal knot slots = numPoints - k.
        // Number of internal segments = (numPoints - k) + 1.
        int internalSegments = static_cast<int>(numPoints) - k + 1;

        if (internalSegments > 0) {
            double step = 1.0 / static_cast<double>(internalSegments);
            for (size_t i = k; i < numPoints; ++i) {
                // i - k + 1 gives 1, 2, 3...
                knots[i] = step * static_cast<double>(i - k + 1);
            }
        }

        return knots;
    }

    /**
     * @brief Evaluates the B-Spline at parameter t using De Boor's Algorithm.
     *
     * @param t The parameter value [0, 1].
     * @param knots The knot vector.
     * @return Vector3 The interpolated point.
     */
    Vector3 EvaluateDeBoor(double t, const std::vector<double>& knots) const {
        int k = m_order;
        int p = k - 1; // Degree

        // 1. Find index 'j' such that knots[j] <= t < knots[j+1]
        // Since we have multiple knots at the end (1.0), we need to be careful with t=1.0.
        // We look for the upper bound.
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        // Clamp j for the exact end case (t=1.0)
        // For quasi-uniform, the domain ends at knots[numPoints].
        // We need to ensure we don't go out of bounds for the basis calculation.
        if (t >= knots[knots.size() - k]) {
            j = static_cast<int>(knots.size()) - k - 1;
        }

        // 2. Initialize temporary control points
        // We need points P[j-p] ... P[j]
        std::vector<Vector3> d;
        d.reserve(k);
        for (int i = 0; i <= p; ++i) {
            // Index safety check (though logic should prevent this)
            int idx = j - p + i;
            if (idx >= 0 && idx < static_cast<int>(m_points.size())) {
                d.push_back(m_points[idx]);
            }
            else {
                d.push_back(Vector3::Zero());
            }
        }

        // 3. De Boor Recursion
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                // Map indices to knot vector
                int index_low = j - p + i;
                int index_high = j + 1 + i - r;

                double denominator = knots[index_high] - knots[index_low];
                double alpha = 0.0;

                // Check for 0 denominator (common in Quasi-Uniform due to duplicate knots)
                if (std::abs(denominator) > 1e-9) {
                    alpha = (t - knots[index_low]) / denominator;
                }

                // Linear interpolation
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p];
    }
};

#endif // QUASI_UNIFORM_BSPLINE_CURVE_H
