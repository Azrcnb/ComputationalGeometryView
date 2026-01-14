#ifndef CARDINAL_CURVE_H
#define CARDINAL_CURVE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Cardinal Spline curve in 3D space.
 *
 * A Cardinal spline is a type of Hermite spline where the tangent at each point
 * is calculated based on the previous and next points, scaled by a tension parameter.
 * This implementation includes phantom points handling to ensure the curve passes
 * through the first and last control points.
 */
class CardinalCurve {
public:
    /**
     * @brief Structure to hold the polynomial coefficients for a single curve segment.
     * Curve equation: P(t) = B1 + B2*t + B3*t^2 + B4*t^3
     */
    struct Coefficients {
        Vector3 B1; // Constant term (t^0)
        Vector3 B2; // Linear term (t^1)
        Vector3 B3; // Quadratic term (t^2)
        Vector3 B4; // Cubic term (t^3)
    };

    CardinalCurve() = default;
    ~CardinalCurve() = default;

    /**
     * @brief Sets the control points for the curve.
     * Sets a dirty flag to trigger recalculation on the next GenerateCurve call.
     *
     * @param points A vector of 3D points (Eigen::Vector3d).
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        m_points = points;
        m_needsUpdate = true;
    }

    /**
     * @brief Sets the tension parameter for the Cardinal spline.
     *
     * @param tension Controls the "tightness" of the curve.
     *                Commonly 0.0 (Catmull-Rom) to 1.0.
     *                The original algorithm snippet used u=0.2.
     */
    void SetTension(double tension) {
        // Only update if value actually changes
        if (std::abs(m_tension - tension) > 1e-6) {
            m_tension = tension;
            m_needsUpdate = true;
        }
    }

    /**
     * @brief Generates a discrete list of vertices representing the curve.
     *
     * Uses lazy evaluation: recalculates coefficients only if points or tension have changed.
     *
     * @param tStep The step size for sampling parameter t (0.0 to 1.0) along each segment.
     * @param outVertices Output vector where the generated 3D points will be stored.
     */
    void GenerateCurve(float tStep, std::vector<Vector3>& outVertices) {
        outVertices.clear();

        // 1. Check if we need to re-compute coefficients (Lazy Evaluation)
        if (m_needsUpdate) {
            if (!ComputeCoefficients()) {
                return; // Not enough points
            }
        }

        if (m_coeffs.empty()) return;

        // Reserve memory roughly
        outVertices.reserve(m_coeffs.size() * static_cast<size_t>(1.0f / tStep));

        // 2. Sample the curve
        for (const auto& segment : m_coeffs) {
            for (float t = 0.0f; t < 1.0f; t += tStep) {
                // Polynomial evaluation: B1 + B2*t + B3*t^2 + B4*t^3
                float t2 = t * t;
                float t3 = t2 * t;

                Vector3 p = segment.B1 +
                    segment.B2 * t +
                    segment.B3 * t2 +
                    segment.B4 * t3;

                outVertices.push_back(p);
            }
        }

        // Add the very last point explicitly to close the gap
        if (!m_points.empty()) {
            outVertices.push_back(m_points.back());
        }
    }

private:
    std::vector<Vector3> m_points;
    std::vector<Coefficients> m_coeffs;
    double m_tension = 0.2; // Default tension matching the snippet's "u=0.2"
    bool m_needsUpdate = false;

    /**
     * @brief Computes the Matrix and Coefficients for all segments.
     * Logic migrated from MultiplyMatrix and DrawCardinalCurve.
     */
    bool ComputeCoefficients() {
        size_t n = m_points.size();
        if (n < 2) return false;

        m_coeffs.clear();

        // Prepare the basis matrix M based on tension
        // Formula from source: s = (1 - u) / 2
        double s = (1.0 - m_tension) / 2.0;

        // Matrix M structure (4x4)
        // In the source snippet:
        // Row 0 corresponds to t^3 coefficient (Point[0])
        // Row 1 corresponds to t^2 coefficient (Point[1])
        // Row 2 corresponds to t^1 coefficient (Point[2])
        // Row 3 corresponds to t^0 coefficient (Point[3])
        Eigen::Matrix4d M;
        M << -s, 2.0 - s, s - 2.0, s,     // Row 0 (t^3)
            2.0 * s, s - 3.0, 3.0 - 2.0 * s, -s,    // Row 1 (t^2)
            -s, 0.0, s, 0.0,   // Row 2 (t^1)
            0.0, 1.0, 0.0, 0.0;   // Row 3 (t^0)

        // Create a temporary list of points with "Phantom" start and end points
        // to ensure the curve passes through all actual control points.
        // We duplicate the first and last points.
        // Original: P0, P1, ..., Pn
        // Extended: P0, P0, P1, ..., Pn, Pn
        std::vector<Vector3> extendedPoints;
        extendedPoints.reserve(n + 2);
        extendedPoints.push_back(m_points.front()); // Phantom start (duplicate P0)
        extendedPoints.insert(extendedPoints.end(), m_points.begin(), m_points.end());
        extendedPoints.push_back(m_points.back());  // Phantom end (duplicate Pn)

        // Calculate coefficients for each segment
        // A segment connects extendedPoints[i+1] and extendedPoints[i+2]
        // But requires neighbors [i] and [i+3] for calculation.
        // The loop iterates through the valid segments defined by the original points.

        size_t numSegments = m_points.size() - 1;

        for (size_t i = 0; i < numSegments; ++i) {
            // P vector contains the 4 control points for this segment calculation
            // Corresponding to: P[i], P[i+1], P[i+2], P[i+3] in a sliding window
            Vector3 P0 = extendedPoints[i];
            Vector3 P1 = extendedPoints[i + 1];
            Vector3 P2 = extendedPoints[i + 2];
            Vector3 P3 = extendedPoints[i + 3];

            // Perform Matrix Multiplication logic from snippet:
            // Point[0] (B4) = M[0][0]*P0 + M[0][1]*P1 + ...
            // We calculate B1, B2, B3, B4 directly using vector math.

            Coefficients segment;

            // Row 0 -> B4 (Coefficient of t^3)
            segment.B4 = M(0, 0) * P0 + M(0, 1) * P1 + M(0, 2) * P2 + M(0, 3) * P3;
            // Row 1 -> B3 (Coefficient of t^2)
            segment.B3 = M(1, 0) * P0 + M(1, 1) * P1 + M(1, 2) * P2 + M(1, 3) * P3;
            // Row 2 -> B2 (Coefficient of t^1)
            segment.B2 = M(2, 0) * P0 + M(2, 1) * P1 + M(2, 2) * P2 + M(2, 3) * P3;
            // Row 3 -> B1 (Coefficient of t^0 / Constant term)
            segment.B1 = M(3, 0) * P0 + M(3, 1) * P1 + M(3, 2) * P2 + M(3, 3) * P3;

            m_coeffs.push_back(segment);
        }

        m_needsUpdate = false;
        return true;
    }
};

#endif // CARDINAL_CURVE_H
