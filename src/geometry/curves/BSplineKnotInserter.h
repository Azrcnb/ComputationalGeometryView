#ifndef BSPLINE_KNOT_INSERTER_H
#define BSPLINE_KNOT_INSERTER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D point representation
using Vector3 = Eigen::Vector3d;

/**
 * @brief Handles Knot Insertion for B-Spline curves (Boehm's Algorithm).
 *
 * Knot insertion adds a new knot to the knot vector without changing the shape
 * of the curve. This results in a new set of control points.
 * This is useful for:
 * 1. Increasing local control (more control points).
 * 2. Splitting curves (subdivision).
 * 3. Converting B-Splines to Piecewise Bezier curves.
 */
class BSplineKnotInserter {
public:
    /**
     * @brief Constructor.
     * @param order The order of the curve (k). Degree p = k - 1.
     * @param points Current control points.
     * @param knots Current knot vector.
     */
    BSplineKnotInserter(int order,
        const std::vector<Vector3>& points,
        const std::vector<double>& knots)
        : m_order(order), m_points(points), m_knots(knots) {
    }

    ~BSplineKnotInserter() = default;

    /**
     * @brief Inserts a knot 'u' multiple times.
     *
     * @param newKnot The knot value to insert.
     * @param multiplicity How many times to insert this knot (r).
     * @return True if insertion was successful, False if knot is outside domain.
     */
    bool InsertKnot(double newKnot, int multiplicity = 1) {
        // Validation: Knot must be within the valid domain [t_{k-1}, t_{n+1}]
        if (m_knots.empty() || m_points.empty()) return false;

        double minKnot = m_knots[m_order - 1];
        double maxKnot = m_knots[m_points.size()];

        if (newKnot < minKnot || newKnot > maxKnot) {
            std::cerr << "Error: Inserted knot is outside the valid domain." << std::endl;
            return false;
        }

        // Perform insertion 'multiplicity' times
        // Iterative approach is numerically stable and cleaner than batch formulas.
        for (int i = 0; i < multiplicity; ++i) {
            PerformSingleInsertion(newKnot);
        }

        return true;
    }

    /**
     * @brief Get the Refined Control Points.
     */
    const std::vector<Vector3>& GetNewControlPoints() const {
        return m_points;
    }

    /**
     * @brief Get the Refined Knot Vector.
     */
    const std::vector<double>& GetNewKnots() const {
        return m_knots;
    }

private:
    int m_order; // Order k
    std::vector<Vector3> m_points; // Stores the evolving control points
    std::vector<double> m_knots;   // Stores the evolving knot vector

    /**
     * @brief Implementation of Boehm's Algorithm for inserting a single knot.
     *
     * @param u The knot value to insert.
     */
    void PerformSingleInsertion(double u) {
        int k = m_order;
        int p = k - 1; // Degree
        size_t n = m_points.size() - 1; // Index of last control point

        // 1. Find the span index 'j' such that knots[j] <= u < knots[j+1]
        // We use upper_bound to find the insertion point in the sorted knot vector.
        auto it = std::upper_bound(m_knots.begin(), m_knots.end(), u);
        int j = static_cast<int>(std::distance(m_knots.begin(), it)) - 1;

        // 2. Create new Control Points (Q)
        // Size increases by 1
        std::vector<Vector3> Q;
        Q.reserve(m_points.size() + 1);

        // Boehm's Rule:
        // Q_i = P_i                       if i <= j - p
        // Q_i = (1-a_i)P_{i-1} + a_i*P_i  if j - p + 1 <= i <= j
        // Q_i = P_{i-1}                   if i >= j + 1

        // Loop through the new indices 'i' from 0 to n+1
        for (int i = 0; i <= static_cast<int>(n) + 1; ++i) {
            if (i <= j - p) {
                // Keep original points
                Q.push_back(m_points[i]);
            }
            else if (i >= j + 1) {
                // Shift original points
                Q.push_back(m_points[i - 1]);
            }
            else {
                // Interpolate (The Core Calculation)
                // Calculate alpha
                // alpha = (u - t_i) / (t_{i+p} - t_i)
                double numerator = u - m_knots[i];
                double denominator = m_knots[i + p] - m_knots[i];

                double alpha = 0.0;
                if (std::abs(denominator) > 1e-9) {
                    alpha = numerator / denominator;
                }

                // Apply formula: Q[i] = (1 - alpha) * P[i-1] + alpha * P[i]
                Vector3 newPoint = (1.0 - alpha) * m_points[i - 1] + alpha * m_points[i];
                Q.push_back(newPoint);
            }
        }

        // 3. Update Knot Vector
        // Simply insert 'u' at the correct position (after index j)
        m_knots.insert(m_knots.begin() + j + 1, u);

        // 4. Update internal state
        m_points = Q;
    }
};

#endif // BSPLINE_KNOT_INSERTER_H
