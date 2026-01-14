#ifndef NURBS_SURFACE_H
#define NURBS_SURFACE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

// Using Eigen's Vector3d for 3D points and Vector4d for Homogeneous coordinates
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;

/**
 * @brief Represents a NURBS (Non-Uniform Rational B-Spline) Surface.
 *
 * FIXED VERSION: Corrected Knot Vector generation logic (Transposition for U-Knots).
 */
class NURBSSurface {
public:
    NURBSSurface(int uOrder = 4, int vOrder = 3)
        : m_uOrder(uOrder), m_vOrder(vOrder) {
    }

    ~NURBSSurface() = default;

    void SetControlData(const std::vector<std::vector<Vector3>>& pointGrid,
        const std::vector<std::vector<double>>& weightGrid = {}) {
        if (pointGrid.empty() || pointGrid[0].empty()) return;

        m_controlPoints = pointGrid;
        m_n = static_cast<int>(pointGrid.size());    // Rows (U direction)
        m_m = static_cast<int>(pointGrid[0].size()); // Cols (V direction)

        // Handle weights
        if (weightGrid.empty()) {
            m_weights.assign(m_n, std::vector<double>(m_m, 1.0));
        }
        else {
            if (weightGrid.size() == m_n && weightGrid[0].size() == m_m) {
                m_weights = weightGrid;
            }
            else {
                std::cerr << "Error: Weight grid dimensions mismatch. Defaulting to 1.0." << std::endl;
                m_weights.assign(m_n, std::vector<double>(m_m, 1.0));
            }
        }

        // Generate Knots
        GenerateGlobalKnots();
    }

    void GenerateSurface(int steps, std::vector<Vector3>& vertices, std::vector<unsigned int>& indices) {
        vertices.clear();
        indices.clear();

        if (m_controlPoints.empty()) return;
        if (steps < 1) steps = 1;

        double stepSize = 1.0 / static_cast<double>(steps);

        for (int i = 0; i <= steps; ++i) {
            double u = i * stepSize;
            if (u > 1.0) u = 1.0;

            for (int j = 0; j <= steps; ++j) {
                double v = j * stepSize;
                if (v > 1.0) v = 1.0;

                vertices.push_back(Evaluate(u, v));
            }
        }

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

    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        lineVertices.clear();
        if (m_controlPoints.empty()) return;

        // U-lines
        for (int i = 0; i < m_n; ++i) {
            for (int j = 0; j < m_m - 1; ++j) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i][j + 1]);
            }
        }
        // V-lines
        for (int j = 0; j < m_m; ++j) {
            for (int i = 0; i < m_n - 1; ++i) {
                lineVertices.push_back(m_controlPoints[i][j]);
                lineVertices.push_back(m_controlPoints[i + 1][j]);
            }
        }
    }

private:
    int m_uOrder;
    int m_vOrder;
    int m_n;      // Rows (U)
    int m_m;      // Cols (V)

    std::vector<std::vector<Vector3>> m_controlPoints;
    std::vector<std::vector<double>> m_weights;

    std::vector<double> m_knotsU;
    std::vector<double> m_knotsV;

    Vector3 Evaluate(double u, double v) const {
        // 1. Evaluate in V direction (Cols) -> use V-Knots
        std::vector<Vector4> tempPoints4D;
        tempPoints4D.reserve(m_n);

        for (int i = 0; i < m_n; ++i) {
            std::vector<Vector4> row4D;
            row4D.reserve(m_m);
            for (int j = 0; j < m_m; ++j) {
                Vector3 p = m_controlPoints[i][j];
                double w = m_weights[i][j];
                row4D.push_back(Vector4(p.x() * w, p.y() * w, p.z() * w, w));
            }
            tempPoints4D.push_back(EvaluateCurveRationalDeBoor(v, m_vOrder, m_knotsV, row4D));
        }

        // 2. Evaluate in U direction (Rows) -> use U-Knots
        Vector4 result4D = EvaluateCurveRationalDeBoor(u, m_uOrder, m_knotsU, tempPoints4D);

        double w = result4D.w();
        if (std::abs(w) < 1e-9) return Vector3::Zero();
        return Vector3(result4D.x() / w, result4D.y() / w, result4D.z() / w);
    }

    Vector4 EvaluateCurveRationalDeBoor(double t, int k, const std::vector<double>& knots, const std::vector<Vector4>& points) const {
        int p = k - 1;

        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        int j = static_cast<int>(std::distance(knots.begin(), it)) - 1;

        if (t >= knots.back()) j = static_cast<int>(points.size()) - 1;

        std::vector<Vector4> d;
        d.reserve(k);
        for (int i = 0; i <= p; ++i) {
            int idx = j - p + i;
            if (idx >= 0 && idx < static_cast<int>(points.size())) d.push_back(points[idx]);
            else d.push_back(Vector4::Zero());
        }

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

    // =========================================================================
    // FIX IS HERE:
    // =========================================================================
    void GenerateGlobalKnots() {
        // 1. Calculate U Knots (Associated with Rows m_n)
        // We MUST Transpose the grid to measure "vertical" lengths (U direction)
        std::vector<std::vector<Vector3>> transposedPoints(m_m, std::vector<Vector3>(m_n));
        for (int i = 0; i < m_n; ++i)
            for (int j = 0; j < m_m; ++j)
                transposedPoints[j][i] = m_controlPoints[i][j];

        // Now transposedPoints has m_m rows, each of length m_n.
        // This generates knots suitable for interpolation of m_n points.
        m_knotsU = ComputeAverageKnots(transposedPoints, m_uOrder);

        // 2. Calculate V Knots (Associated with Cols m_m)
        // Pass original grid -> measures lines of length m_m.
        m_knotsV = ComputeAverageKnots(m_controlPoints, m_vOrder);
    }

    std::vector<double> ComputeAverageKnots(const std::vector<std::vector<Vector3>>& grid, int k) {
        if (grid.empty()) return {};

        int numLines = static_cast<int>(grid.size());
        int ptsPerLine = static_cast<int>(grid[0].size());

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
                for (int i = 0; i < ptsPerLine; ++i) avgParams[i] += (double)i / (ptsPerLine - 1);
            }
        }
        for (double& p : avgParams) p /= numLines;

        int numKnots = ptsPerLine + k;
        std::vector<double> knots(numKnots);
        int p = k - 1;

        for (int i = 0; i < k; ++i) knots[i] = 0.0;
        for (int i = ptsPerLine; i < numKnots; ++i) knots[i] = 1.0;

        for (int j = 1; j < ptsPerLine - p; ++j) {
            double sum = 0.0;
            for (int m = j; m < j + p; ++m) sum += avgParams[m];
            knots[j + k - 1] = sum / p;
        }
        return knots;
    }
};

#endif // NURBS_SURFACE_H
