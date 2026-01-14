#ifndef COMPOSITE_BEZIER_CURVE_H
#define COMPOSITE_BEZIER_CURVE_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a chain of spliced Cubic Bezier curves (Composite Bezier).
 *
 * Instead of creating one high-degree curve for all points, this class interprets
 * the control points as a sequence of connected cubic Bezier segments.
 *
 * Rule for Control Points:
 * To maintain continuity (C0), the end point of segment N is the start point of segment N+1.
 * Therefore, the input vector size must satisfy: Size = 3 * k + 1 (where k is the number of segments).
 * Indices for Segment i: [3*i, 3*i+1, 3*i+2, 3*i+3]
 */
class CompositeBezierCurve {
public:
    /**
     * @brief Structure representing a single Cubic Bezier segment (4 control points).
     */
    struct Segment {
        Vector3 P0; // Start Anchor
        Vector3 P1; // Start Handle
        Vector3 P2; // End Handle
        Vector3 P3; // End Anchor
    };

    CompositeBezierCurve() = default;
    ~CompositeBezierCurve() = default;

    /**
     * @brief Sets the control points for the composite curve.
     *
     * @param points A vector of 3D points.
     *                Ideally, size should be 4, 7, 10, 13, etc.
     *                If size is not (3k+1), extra points at the end will be ignored.
     */
    void SetControlPoints(const std::vector<Vector3>& points) {
        m_rawPoints = points;
        m_needsUpdate = true; // Mark as dirty
    }

    /**
     * @brief Generates a discrete list of vertices representing the entire spliced curve.
     *
     * @param tStep The step size for sampling parameter t (0.0 to 1.0) WITHIN EACH SEGMENT.
     *              Smaller values result in a smoother curve.
     * @param outVertices Output vector where the generated 3D points will be stored.
     */
    void GenerateCurve(float tStep, std::vector<Vector3>& outVertices) {
        outVertices.clear();

        // 1. Check if we need to process segments (Lazy Evaluation)
        if (m_needsUpdate) {
            if (!BuildSegments()) {
                return; // Not enough points to form even one segment
            }
        }

        if (m_segments.empty()) return;

        // Reserve memory roughly (segments * steps)
        outVertices.reserve(m_segments.size() * static_cast<size_t>(1.0f / tStep));

        // 2. Iterate through each segment and sample it
        for (const auto& seg : m_segments) {
            // Sample t from 0 to 1 for this specific segment
            // Note: We use < 1.0 to avoid duplicating the join points
            // (The end of Segment 0 is the start of Segment 1)
            for (float t = 0.0f; t < 1.0f; t += tStep) {
                outVertices.push_back(EvaluateDeCasteljau(seg, t));
            }
        }

        // 3. Add the very last anchor point of the entire chain
        if (!m_segments.empty()) {
            outVertices.push_back(m_segments.back().P3);
        }
    }

private:
    std::vector<Vector3> m_rawPoints;
    std::vector<Segment> m_segments;
    bool m_needsUpdate = false;

    /**
     * @brief Parses the raw points into Cubic Bezier Segments.
     * Logic: Takes points 0,1,2,3 for Seg 1; 3,4,5,6 for Seg 2, etc.
     */
    bool BuildSegments() {
        m_segments.clear();
        size_t n = m_rawPoints.size();

        // We need at least 4 points to make one cubic curve
        if (n < 4) return false;

        // Calculate number of segments
        // Formula: (N - 1) / 3
        size_t numSegments = (n - 1) / 3;

        m_segments.reserve(numSegments);

        for (size_t i = 0; i < numSegments; ++i) {
            size_t baseIdx = i * 3;
            Segment seg;
            seg.P0 = m_rawPoints[baseIdx];
            seg.P1 = m_rawPoints[baseIdx + 1];
            seg.P2 = m_rawPoints[baseIdx + 2];
            seg.P3 = m_rawPoints[baseIdx + 3];
            m_segments.push_back(seg);
        }

        m_needsUpdate = false;
        return true;
    }

    /**
     * @brief Evaluates a specific cubic segment at t using De Casteljau's algorithm.
     *
     * @param seg The segment containing 4 control points.
     * @param t The interpolation parameter [0, 1].
     * @return The 3D point on the curve.
     */
    Vector3 EvaluateDeCasteljau(const Segment& seg, float t) const {
        double u = 1.0 - t;

        // Layer 1: Interpolate between control points
        Vector3 q0 = u * seg.P0 + t * seg.P1;
        Vector3 q1 = u * seg.P1 + t * seg.P2;
        Vector3 q2 = u * seg.P2 + t * seg.P3;

        // Layer 2: Interpolate between Q points
        Vector3 r0 = u * q0 + t * q1;
        Vector3 r1 = u * q1 + t * q2;

        // Layer 3: Interpolate between R points (Final Result)
        Vector3 point = u * r0 + t * r1;

        return point;
    }
};

#endif // COMPOSITE_BEZIER_CURVE_H
