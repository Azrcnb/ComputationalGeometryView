#ifndef NURBS_CIRCLE_APPROXIMATOR_H
#define NURBS_CIRCLE_APPROXIMATOR_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D point and Vector4d for Homogeneous coordinates
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief A specialized class for approximating Circular Arcs using NURBS.
 *
 * This class encapsulates the logic to generate Control Points, Weights, and Knots
 * required to EXACTLY represent a circular arc given a Radius, Start Angle, and Sweep Angle.
 *
 * It implements the logic found in the source examples for various angular spans
 * (e.g., small arcs < 90 deg, large arcs up to 360 deg).
 */
class NURBSCircleApproximator {
public:
    NURBSCircleApproximator() : m_order(3) {} // Default to Quadratic (k=3) for circles
    ~NURBSCircleApproximator() = default;

    /**
     * @brief Setup the NURBS data for a circular arc.
     *
     * This function analyzes the sweep angle and automatically determines the
     * number of segments required (since a single rational Bezier segment can
     * typically only cover up to 90-120 degrees well without numerical issues,
     * and strictly speaking 90 degrees is the standard building block for circles).
     *
     * @param radius The radius of the arc.
     * @param startAngleRad Start angle in Radians.
     * @param sweepAngleRad Sweep angle in Radians (Total angle).
     */
    void SetupArc(double radius, double startAngleRad, double sweepAngleRad) {
        m_points.clear();
        m_weights.clear();
        m_knots.clear();

        // Standard Circle Representation usually uses Degree 2 (Order 3)
        m_order = 3;

        // 1. Determine number of segments needed.
        // A common strategy is to split arcs > 90 degrees (PI/2).
        // However, the source code examples show specific manual constructions 
        // for 150, 240, 300 degrees using different weighting schemes.
        // To be robust and match the "Approximation" logic generalized from the examples:

        // Let's implement the specific logic for the provided examples if they match,
        // otherwise default to a generic segmentation approach.

        // --- Strategy: Generic Segmentation ---
        // Any arc can be split into 'numSegments' pieces where each piece <= 90 degrees.
        int numSegments = static_cast<int>(std::ceil(std::abs(sweepAngleRad) / (M_PI / 2.0)));
        if (numSegments < 1) numSegments = 1;

        double thetaPerSeg = sweepAngleRad / numSegments;

        // Total Control Points = 2 * numSegments + 1  (For Quadratic)
        // Knots: Clamped at ends, internal knots with multiplicity 'p' (order-1)
        // But for standard NURBS circle continuity C1, we can optimize.
        // However, let's stick to the pattern in the source:
        // Source 240 deg (4 segments of 60? No, let's look at knots):
        // Knots: 0,0,0, 1/3, 1/3, 2/3, 2/3, 1,1,1 -> Multiplicity 2 internal -> C0 continuity?
        // Actually, quadratic NURBS with double knots acts like piecewise Bezier.

        // Let's implement the generic "Piecewise Rational Quadratic Bezier" approach
        // which matches the source code structure (internal knots repeated).

        // Knots
        // Start: 0, 0, 0 (Order 3)
        for (int i = 0; i < m_order; ++i) m_knots.push_back(0.0);

        // Internal knots
        for (int i = 1; i < numSegments; ++i) {
            double val = static_cast<double>(i) / static_cast<double>(numSegments);
            // Repeat 'order-1' times for Piecewise Bezier (matches source logic of 1/3, 1/3)
            for (int j = 0; j < m_order - 1; ++j) m_knots.push_back(val);
        }

        // End: 1, 1, 1
        for (int i = 0; i < m_order; ++i) m_knots.push_back(1.0);

        // Control Points & Weights
        // We build each segment locally and append.
        // A rational quadratic Bezier arc of angle 'theta':
        // P0 = (r, 0)
        // P1 = (r, r*tan(theta/2)) -> Projective weight cos(theta/2)
        // P2 = (r*cos(theta), r*sin(theta))

        // Current angle
        double currentPhi = startAngleRad;

        // Add absolute first point
        m_points.push_back(Vector3(radius * std::cos(currentPhi), radius * std::sin(currentPhi), 0.0));
        m_weights.push_back(1.0);

        for (int i = 0; i < numSegments; ++i) {
            double nextPhi = currentPhi + thetaPerSeg;

            // Mid angle for P1 construction
            double halfTheta = thetaPerSeg / 2.0;
            double midPhi = currentPhi + halfTheta;

            // Weight for the middle control point of this segment
            double segmentWeight = std::cos(halfTheta);

            // P1 (Middle Control Point of segment)
            // Distance from center to P1 is r / cos(halfTheta)
            double r1 = radius / std::cos(halfTheta);
            Vector3 P1(r1 * std::cos(midPhi), r1 * std::sin(midPhi), 0.0);

            // P2 (End Control Point of segment)
            Vector3 P2(radius * std::cos(nextPhi), radius * std::sin(nextPhi), 0.0);

            // Append
            m_points.push_back(P1);
            m_weights.push_back(segmentWeight); // Matches W[1] = cos(Theta/2) logic

            m_points.push_back(P2);
            m_weights.push_back(1.0); // End point of segment always weight 1

            currentPhi = nextPhi;
        }
    }

    /**
     * @brief Generates the discrete vertices of the curve.
     * Uses Rational De Boor evaluation.
     */
    void GenerateCurve(double sampleResolution, std::vector<Vector3>& outVertices) const {
        outVertices.clear();
        if (m_points.empty()) return;

        double tStart = 0.0;
        double tEnd = 1.0;

        size_t estimatedPoints = static_cast<size_t>((tEnd - tStart) / sampleResolution) + 2;
        outVertices.reserve(estimatedPoints);

        for (double t = tStart; t <= tEnd; t += sampleResolution) {
            outVertices.push_back(EvaluateRationalDeBoor(t));
        }

        // Ensure endpoint
        Vector3 endP = EvaluateRationalDeBoor(tEnd);
        if (outVertices.empty() || (outVertices.back() - endP).norm() > 1e-6) {
            outVertices.push_back(endP);
        }
    }

    // Accessors for debugging
    const std::vector<Vector3>& GetControlPoints() const { return m_points; }
    const std::vector<double>& GetWeights() const { return m_weights; }

private:
    int m_order; // Order k (Standard 3 for Quadratic arcs)
    std::vector<Vector3> m_points;
    std::vector<double> m_weights;
    std::vector<double> m_knots;

    /**
     * @brief Evaluates curve at parameter t using Rational De Boor.
     */
    Vector3 EvaluateRationalDeBoor(double t) const {
        int k = m_order;
        int p = k - 1;

        // Find span
        auto it = std::upper_bound(m_knots.begin(), m_knots.end(), t);
        int j = static_cast<int>(std::distance(m_knots.begin(), it)) - 1;

        if (t >= m_knots[m_points.size()]) {
            j = static_cast<int>(m_points.size()) - 1;
        }

        // Initialize Homogeneous Points
        std::vector<Vector4> d;
        d.reserve(k);
        for (int i = 0; i <= p; ++i) {
            int idx = j - p + i;
            if (idx >= 0 && idx < static_cast<int>(m_points.size())) {
                double w = m_weights[idx];
                d.push_back(Vector4(m_points[idx].x() * w, m_points[idx].y() * w, m_points[idx].z() * w, w));
            }
            else {
                d.push_back(Vector4::Zero());
            }
        }

        // De Boor Loop
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                int index_low = j - p + i;
                int index_high = j + 1 + i - r;
                double denominator = m_knots[index_high] - m_knots[index_low];
                double alpha = (std::abs(denominator) > 1e-9) ? ((t - m_knots[index_low]) / denominator) : 0.0;
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        Vector4 res = d[p];
        if (std::abs(res.w()) < 1e-9) return Vector3::Zero();
        return Vector3(res.x() / res.w(), res.y() / res.w(), res.z() / res.w());
    }
};

#endif // NURBS_CIRCLE_APPROXIMATOR_H
