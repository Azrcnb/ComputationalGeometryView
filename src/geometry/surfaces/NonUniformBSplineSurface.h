#ifndef NON_UNIFORM_BSPLINE_SURFACE_H
#define NON_UNIFORM_BSPLINE_SURFACE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a Non-Uniform B-Spline Surface (NUBS Surface).
 *
 * This class implements a tensor product B-Spline surface where the knot vectors
 * are determined by the geometry of the control net (Hartley-Judd Algorithm / Chord Length).
 * This allows the surface to better approximate unevenly spaced control grids.
 */
class NonUniformBSplineSurface {
public:
    /**
     * @brief Constructor.
     * @param uOrder Order in U direction (p+1). Default 4 (Cubic).
     * @param vOrder Order in V direction (q+1). Default 3 (Quadratic).
     */
    NonUniformBSplineSurface(int uOrder = 4, int vOrder = 3)
        : m_uOrder(uOrder), m_vOrder(vOrder) {
    }

    ~NonUniformBSplineSurface() = default;

    /**
     * @brief Sets the Control Point Grid.
     *
     * @param grid A 2D vector where grid[i][j] is the point at row i (u), col j (v).
     *             Dimensions must be at least [uOrder] x [vOrder].
     */
    void SetControlGrid(const std::vector<std::vector<Vector3>>& grid) {
        if (grid.empty() || grid[0].empty()) return;

        m_controlPoints = grid;
        m_n = static_cast<int>(grid.size());    // Rows (U direction size)
        m_m = static_cast<int>(grid[0].size()); // Cols (V direction size)

        // Generate Knot Vectors using Hartley-Judd Algorithm
        // Note: In a rigorous tensor product surface, we typically need ONE knot vector per direction.
        // However, the source code implies calculating knots PER ROW/COL or averaging them.
        // Standard approach: Average the chord lengths across all rows to get one U-knot vector,
        // and across all cols to get one V-knot vector.
        // The source code seems to re-calculate knots for every evaluation which is inefficient.
        // Here we generate global U and V knot vectors by averaging the geometric distribution.

        GenerateGlobalKnots();
    }

    /**
     * @brief Generates mesh data for OpenGL rendering.
     *
     * @param steps Tessellation level (e.g., 20x20).
     * @param vertices Output vector for vertices.
     * @param indices Output vector for triangle indices.
     */
    void GenerateSurface(int steps, std::vector<Vector3>& vertices, std::vector<unsigned int>& indices) {
        vertices.clear();
        indices.clear();

        if (m_controlPoints.empty()) return;
        if (steps < 1) steps = 1;

        // Valid Domain is [0, 1] due to our normalization
        double stepSize = 1.0 / static_cast<double>(steps);

        // 1. Generate Vertices
        for (int i = 0; i <= steps; ++i) {
            double u = i * stepSize;
            // Clamp to handle floating point errors at 1.0
            if (u > 1.0) u = 1.0;

            for (int j = 0; j <= steps; ++j) {
                double v = j * stepSize;
                if (v > 1.0) v = 1.0;

                vertices.push_back(Evaluate(u, v));
            }
        }

        // 2. Generate Indices (Quads -> 2 Triangles)
        int rowLength = steps + 1;
        for (int i = 0; i < steps; ++i) {
            for (int j = 0; j < steps; ++j) {
                unsigned int p0 = i * rowLength + j;
                unsigned int p1 = p0 + 1;
                unsigned int p2 = (i + 1) * rowLength + j;
                unsigned int p3 = p2 + 1;

                indices.push_back(p0);
                indices.push_back(p2);
                indices.push_back(p1);

                indices.push_back(p1);
                indices.push_back(p2);
                indices.push_back(p3);
            }
        }
    }

    /**
     * @brief Generates the control net lines for visualization.
     */
    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        lineVertices.clear();
        if (m_controlPoints.empty()) return;

        // U-lines (Horizontal)
        for (int i = 0; i < m_n; ++i) {
            for (int j = 0; j < m_m - 1; ++j) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i][j + 1]);
            }
        }

        // V-lines (Vertical)
        for (int j = 0; j < m_m; ++j) {
            for (int i = 0; i < m_n - 1; ++i) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i + 1][j]);
            }
        }
    }

private:
    int m_uOrder; // Order p+1
    int m_vOrder; // Order q+1
    int m_n;      // Number of rows
    int m_m;      // Number of cols

    std::vector<std::vector<Vector3>> m_controlPoints;
    std::vector<double> m_knotsU;
    std::vector<double> m_knotsV;

    /**
     * @brief Evaluate surface point S(u,v) using tensor product De Boor.
     *
     * Algorithm:
     * 1. For each row i, evaluate the curve C_i(v) at parameter v.
     *    This gives a set of temporary points Q_i.
     * 2. Evaluate the curve defined by Q_i at parameter u.
     */
    Vector3 Evaluate(double u, double v) const {
        // 1. Evaluate in V direction for each row to get temporary control points
        std::vector<Vector3> tempPoints;
        tempPoints.reserve(m_n);

        for (int i = 0; i < m_n; ++i) {
            // Extract row i
            const std::vector<Vector3>& rowPoints = m_controlPoints[i];
            // Evaluate curve at v using this row
            tempPoints.push_back(EvaluateCurveDeBoor(v, m_vOrder, m_knotsV, rowPoints));
        }

        // 2. Evaluate in U direction using the temp points
        return EvaluateCurveDeBoor(u, m_uOrder, m_knotsU, tempPoints);
    }

    /**
     * @brief Standard De Boor evaluation for a curve segment.
     */
    Vector3 EvaluateCurveDeBoor(double t, int k, const std::vector<double>& knots, const std::vector<Vector3>& points) const {
        int p = k - 1; // Degree

        // Find span
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        // Clamp for end case
        if (t >= knots[points.size()]) {
            j = static_cast<int>(points.size()) - 1;
        }

        // Local control points
        std::vector<Vector3> d;
        d.reserve(k);
        for (int i = 0; i <= p; ++i) {
            int idx = j - p + i;
            if (idx >= 0 && idx < static_cast<int>(points.size()))
                d.push_back(points[idx]);
            else
                d.push_back(Vector3::Zero());
        }

        // De Boor Loop
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                int index_low = j - p + i;
                int index_high = j + 1 + i - r;
                double denominator = knots[index_high] - knots[index_low];
                double alpha = (std::abs(denominator) > 1e-9) ? ((t - knots[index_low]) / denominator) : 0.0;
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }
        return d[p];
    }

    /**
     * @brief Generate Global Knot Vectors based on average chord lengths.
     *
     * The Hartley-Judd logic in the source code calculates weights based on distance.
     * For a surface, we average these distances to create a single parameterization
     * that works reasonably well for the whole grid.
     */
    void GenerateGlobalKnots() {
        // --- Generate Knots U (based on rows) ---
        m_knotsU = ComputeAverageKnots(m_controlPoints, m_uOrder, true);

        // --- Generate Knots V (based on cols) ---
        // Transpose conceptually for V
        std::vector<std::vector<Vector3>> transposedPoints(m_m, std::vector<Vector3>(m_n));
        for (int i = 0; i < m_n; ++i)
            for (int j = 0; j < m_m; ++j)
                transposedPoints[j][i] = m_controlPoints[i][j];

        m_knotsV = ComputeAverageKnots(transposedPoints, m_vOrder, false);
    }

    std::vector<double> ComputeAverageKnots(const std::vector<std::vector<Vector3>>& grid, int k, bool isU) {
        int numLines = static_cast<int>(grid.size());
        int ptsPerLine = static_cast<int>(grid[0].size());

        // Sum of parameters for each index
        std::vector<double> avgParams(ptsPerLine, 0.0);

        for (const auto& line : grid) {
            double totalLen = 0.0;
            std::vector<double> params(ptsPerLine);
            params[0] = 0.0;

            for (int i = 1; i < ptsPerLine; ++i) {
                totalLen += (line[i] - line[i - 1]).norm();
                params[i] = totalLen;
            }

            if (totalLen > 1e-9) {
                for (int i = 0; i < ptsPerLine; ++i) avgParams[i] += (params[i] / totalLen);
            }
            else {
                // Degenerate line
                for (int i = 0; i < ptsPerLine; ++i) avgParams[i] += (double)i / (ptsPerLine - 1);
            }
        }

        // Average
        for (double& p : avgParams) p /= numLines;

        // Construct Knots from Averaged Parameters (Hartley-Judd / De Boor mapping)
        int numKnots = ptsPerLine + k;
        std::vector<double> knots(numKnots);
        int p = k - 1;

        // Clamped Ends
        for (int i = 0; i < k; ++i) knots[i] = 0.0;
        for (int i = ptsPerLine; i < numKnots; ++i) knots[i] = 1.0;

        // Internal Knots mapping
        for (int j = 1; j < ptsPerLine - p; ++j) {
            double sum = 0.0;
            for (int m = j; m < j + p; ++m) sum += avgParams[m];
            knots[j + k - 1] = sum / p;
        }

        return knots;
    }
};

#endif // NON_UNIFORM_BSPLINE_SURFACE_H
