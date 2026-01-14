#ifndef NURBS_CURVE_H
#define NURBS_CURVE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D points and Vector4d for Homogeneous coordinates
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;

/**
 * @brief Represents a NURBS (Non-Uniform Rational B-Spline) Curve in 3D space.
 *
 * NURBS curves are the industry standard for geometry representation.
 * They generalize B-Splines by adding weights to control points, allowing for:
 * 1. Exact representation of conic sections (circles, ellipses).
 * 2. More control over the curve's influence (pulling curve towards points with high weights).
 *
 * This implementation uses the Rational De Boor algorithm for evaluation
 * and Hartley-Judd (Chord Length) parameterization for knot generation.
 */
class NURBSCurve {
public:
    /**
     * @brief Constructor.
     * @param order The order of the curve (k). Degree p = k - 1.
     *              Standard for many CAD applications is k=4 (Cubic).
     */
    explicit NURBSCurve(int order = 4) : m_order(order) {}

    ~NURBSCurve() = default;

    /**
     * @brief Sets the order of the curve (k).
     * @param k The order (must be >= 2).
     */
    void SetOrder(int k) {
        if (k < 2) return;
        m_order = k;
    }

    /**
     * @brief Sets the control points and their corresponding weights.
     *
     * @param points A vector of 3D control points.
     * @param weights A vector of scalar weights. Must have same size as points.
     *                If empty, weights default to 1.0 (Standard B-Spline).
     */
    void SetControlData(const std::vector<Vector3>& points, const std::vector<double>& weights = {}) {
        m_points = points;

        // Handle weights
        if (weights.empty()) {
            m_weights.assign(points.size(), 1.0);
        }
        else if (weights.size() != points.size()) {
            std::cerr << "Error: Number of weights must match number of control points." << std::endl;
            // Fallback to 1.0
            m_weights.assign(points.size(), 1.0);
        }
        else {
            m_weights = weights;
        }
    }

    /**
     * @brief Generates the discrete vertices of the NURBS curve.
     *
     * 1. Generates Non-Uniform knots based on Chord Length (Hartley-Judd).
     * 2. Evaluates curve using Rational De Boor algorithm.
     *
     * @param sampleResolution The step size for the parameter t [0, 1].
     * @param outVertices Output vector where generated 3D points are stored.
     */
    void GenerateCurve(double sampleResolution, std::vector<Vector3>& outVertices) const {
        outVertices.clear();

        if (m_points.empty() || m_points.size() < static_cast<size_t>(m_order)) {
            return;
        }

        // 1. Generate Knot Vector (Hartley-Judd / Chord Length)
        std::vector<double> knots = GenerateChordLengthKnots(m_points, m_order);

        // 2. Determine Valid Domain [u_{k-1}, u_{n+1}]
        // Since we use Clamped knots (multiplicity k at ends), domain is [0, 1].
        double tStart = 0.0;
        double tEnd = 1.0;

        // 3. Evaluate
        size_t estimatedPoints = static_cast<size_t>((tEnd - tStart) / sampleResolution) + 2;
        outVertices.reserve(estimatedPoints);

        for (double t = tStart; t <= tEnd; t += sampleResolution) {
            outVertices.push_back(EvaluateRationalDeBoor(t, knots));
        }

        // Ensure exact end point
        Vector3 endPoint = EvaluateRationalDeBoor(tEnd, knots);
        if (outVertices.empty() || (outVertices.back() - endPoint).norm() > 1e-6) {
            outVertices.push_back(endPoint);
        }
    }

private:
    int m_order; // Order k
    std::vector<Vector3> m_points;
    std::vector<double> m_weights;

    /**
     * @brief Generates knots based on Chord Length Parameterization (Hartley-Judd).
     *
     * This ensures the parameter distribution matches the geometric distribution of points.
     */
    std::vector<double> GenerateChordLengthKnots(const std::vector<Vector3>& points, int k) const {
        size_t numPoints = points.size();
        size_t numKnots = numPoints + k;
        std::vector<double> knots(numKnots);
        int p = k - 1;

        // 1. Calculate accumulated chord lengths (parameters)
        std::vector<double> params(numPoints);
        params[0] = 0.0;
        double totalLength = 0.0;

        for (size_t i = 1; i < numPoints; ++i) {
            totalLength += (points[i] - points[i - 1]).norm();
            params[i] = totalLength;
        }

        // Normalize to [0, 1]
        if (totalLength > 0.0) {
            for (double& val : params) val /= totalLength;
        }
        else {
            std::fill(params.begin(), params.end(), 0.0);
            params.back() = 1.0;
        }

        // 2. Construct Knot Vector
        // Clamped Start
        for (int i = 0; i < k; ++i) knots[i] = 0.0;
        // Clamped End
        for (size_t i = numPoints; i < numKnots; ++i) knots[i] = 1.0;

        // Internal Knots (Averaging based on parameters)
        // knot[j+k-1] roughly corresponds to param[j] smoothed over p neighbors
        for (size_t j = 1; j < numPoints - p; ++j) {
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
     * @brief Evaluates NURBS at parameter t using Rational De Boor Algorithm.
     *
     * 1. Converts Control Points to Homogeneous Coordinates (x*w, y*w, z*w, w).
     * 2. Performs De Boor interpolation in 4D.
     * 3. Projects result back to 3D (x'/w', y'/w', z'/w').
     */
    Vector3 EvaluateRationalDeBoor(double t, const std::vector<double>& knots) const {
        int k = m_order;
        int p = k - 1;

        // 1. Find knot span
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        if (t >= knots[knots.size() - k]) {
            j = static_cast<int>(knots.size()) - k - 1;
        }

        // 2. Initialize temporary Homogeneous Control Points (4D)
        std::vector<Vector4> d;
        d.reserve(k);

        for (int i = 0; i <= p; ++i) {
            int idx = j - p + i;
            if (idx >= 0 && idx < static_cast<int>(m_points.size())) {
                double w = m_weights[idx];
                Vector3 p3 = m_points[idx];
                // Store as (w*x, w*y, w*z, w)
                d.push_back(Vector4(p3.x() * w, p3.y() * w, p3.z() * w, w));
            }
            else {
                d.push_back(Vector4::Zero());
            }
        }

        // 3. De Boor Recursion in 4D
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                int index_low = j - p + i;
                int index_high = j + 1 + i - r;
                double denominator = knots[index_high] - knots[index_low];
                double alpha = 0.0;

                if (std::abs(denominator) > 1e-9) {
                    alpha = (t - knots[index_low]) / denominator;
                }

                // Interpolate 4D vector
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        // 4. Perspective Division (Project back to 3D)
        Vector4 result4 = d[p];
        double w_final = result4.w();

        if (std::abs(w_final) < 1e-9) return Vector3::Zero();

        return Vector3(result4.x() / w_final, result4.y() / w_final, result4.z() / w_final);
    }
};

#endif // NURBS_CURVE_H
