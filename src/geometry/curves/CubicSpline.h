#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Cubic Spline curve in 3D space.
 *
 * Implements a standard cubic spline interpolation with clamped boundary conditions.
 * The calculation is performed lazily inside GenerateCurve.
 */
class CubicSpline {
public:
    struct Coefficients {
        Vector3 B1, B2, B3, B4; // Polynomial coefficients
        double length;          // Length of the segment
    };

    CubicSpline() = default;
    ~CubicSpline() = default;

    /**
     * @brief Sets the control points and boundary conditions.
     * Sets a dirty flag to trigger recalculation on the next GenerateCurve call.
     *
     * @param points A vector of 3D points.
     * @param startTangentMag Magnitude of start tangent (default 1.0).
     * @param endTangentMag Magnitude of end tangent (default 1.0).
     */
    void SetControlPoints(const std::vector<Vector3>& points,
        double startTangentMag = 1.0,
        double endTangentMag = 1.0) {
        m_points = points;
        m_startTangentMag = startTangentMag;
        m_endTangentMag = endTangentMag;
        m_needsUpdate = true; // Mark as dirty
    }

    /**
     * @brief Generates a discrete list of vertices representing the curve.
     *
     * If control points have changed, this method automatically re-runs
     * the cubic spline coefficient calculation (Thomas Algorithm) before sampling.
     *
     * @param tStep The step size for sampling parameter t.
     * @param outVertices Output vector for 3D points.
     */
    void GenerateCurve(float tStep, std::vector<Vector3>& outVertices) {
        outVertices.clear();

        // 1. Check if we need to re-compute coefficients (Lazy Evaluation)
        if (m_needsUpdate) {
            if (!ComputeCoefficients()) {
                return; // Computation failed (e.g. < 2 points)
            }
        }

        if (m_coeffs.empty()) return;

        // 2. Sample the curve
        outVertices.reserve(m_coeffs.size() * 20);

        for (const auto& coeff : m_coeffs) {
            // Iterate along the segment length
            for (float t = 0.0f; t <= coeff.length; t += tStep) {
                Vector3 p = coeff.B1 +
                    coeff.B2 * t +
                    coeff.B3 * (t * t) +
                    coeff.B4 * (t * t * t);
                outVertices.push_back(p);
            }
        }

        // Ensure the very last point is included
        if (!m_points.empty()) {
            outVertices.push_back(m_points.back());
        }
    }

private:
    std::vector<Vector3> m_points;
    std::vector<Coefficients> m_coeffs;

    // Boundary conditions
    double m_startTangentMag = 1.0;
    double m_endTangentMag = 1.0;

    // Dirty flag for lazy evaluation
    bool m_needsUpdate = false;

    /**
     * @brief Internal helper to solve the Tridiagonal Matrix System.
     * This logic was previously in the public Compute() method.
     */
    bool ComputeCoefficients() {
        size_t n = m_points.size();
        if (n < 2) return false;

        size_t numSegments = n - 1;
        m_coeffs.resize(numSegments);

        // 1. Calculate segment lengths
        std::vector<double> L(numSegments);
        std::vector<Vector3> delta(numSegments);

        for (size_t i = 0; i < numSegments; ++i) {
            delta[i] = m_points[i + 1] - m_points[i];
            L[i] = delta[i].norm();
            if (L[i] < 1e-9) L[i] = 1e-9;
        }

        // 2. Define Boundary Conditions (Clamped)
        Vector3 c1 = delta[0].normalized() * m_startTangentMag;
        Vector3 cn = delta[numSegments - 1].normalized() * m_endTangentMag;

        // 3. Solve Tridiagonal Matrix (Thomas Algorithm)
        size_t numKnots = n;
        std::vector<double> lambda(numKnots, 0.0);
        std::vector<double> mu(numKnots, 0.0);
        std::vector<Vector3> D(numKnots);

        // Internal knots
        for (size_t i = 1; i < numKnots - 1; ++i) {
            lambda[i] = L[i - 1] / (L[i - 1] + L[i]);
            mu[i] = L[i] / (L[i - 1] + L[i]);
            Vector3 term1 = delta[i] / L[i];
            Vector3 term2 = delta[i - 1] / L[i - 1];
            D[i] = 6.0 * (term1 - term2) / (L[i - 1] + L[i]);
        }

        // Boundary conditions
        D[0] = 6.0 * (delta[0] / L[0] - c1) / L[0];
        lambda[0] = 1.0;
        mu[0] = 1.0;
        D[numKnots - 1] = 6.0 * (cn - delta[numSegments - 1] / L[numSegments - 1]) / L[numSegments - 1];

        // Forward Elimination
        std::vector<double> l_arr(numKnots), u_arr(numKnots);
        std::vector<Vector3> K_arr(numKnots), M(numKnots);

        l_arr[0] = 2.0;
        u_arr[0] = 1.0 / l_arr[0];
        K_arr[0] = D[0] / l_arr[0];

        for (size_t i = 1; i < numKnots - 1; ++i) {
            double m_val = lambda[i];
            l_arr[i] = 2.0 - m_val * u_arr[i - 1];
            u_arr[i] = mu[i] / l_arr[i];
            K_arr[i] = (D[i] - K_arr[i - 1] * m_val) / l_arr[i];
        }

        double m_last = 1.0;
        l_arr[numKnots - 1] = 2.0 - m_last * u_arr[numKnots - 2];
        K_arr[numKnots - 1] = (D[numKnots - 1] - K_arr[numKnots - 2] * m_last) / l_arr[numKnots - 1];

        // Backward Substitution
        M[numKnots - 1] = K_arr[numKnots - 1];
        for (int i = (int)numKnots - 2; i >= 0; --i) {
            M[i] = K_arr[i] - M[i + 1] * u_arr[i];
        }

        // 4. Compute Coefficients
        for (size_t i = 0; i < numSegments; ++i) {
            m_coeffs[i].B1 = m_points[i];
            m_coeffs[i].B2 = delta[i] / L[i] - L[i] * (M[i] / 3.0 + M[i + 1] / 6.0);
            m_coeffs[i].B3 = M[i] / 2.0;
            m_coeffs[i].B4 = (M[i + 1] - M[i]) / (6.0 * L[i]);
            m_coeffs[i].length = L[i];
        }

        m_needsUpdate = false; // Calculation complete
        return true;
    }
};

#endif // CUBIC_SPLINE_H
