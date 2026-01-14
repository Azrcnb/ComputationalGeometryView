#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>
#include "CubicBezierPatch.h" // 假设之前的单片类保存在这里

// Using Eigen's Vector3d as the standard 3D point type
using Vector3 = Eigen::Vector3d;

/**
 * @brief Represents a complex surface composed of multiple stitched Cubic Bezier Patches.
 *
 * This class manages a global list of control points and a list of patch definitions.
 * Each patch definition consists of a 4x4 grid of indices referencing the global points.
 * This allows for sharing control points between adjacent patches to ensure continuity (C0/G0).
 */
class CompositeBezierSurface {
public:
    /**
     * @brief Definition of a single patch within the composite surface.
     * Contains 16 indices (4x4) pointing to the global control point list.
     */
    struct PatchDefinition {
        int indices[4][4];
    };

    CompositeBezierSurface() = default;
    ~CompositeBezierSurface() = default;

    /**
     * @brief Sets the global list of control points available to all patches.
     *
     * @param points Vector of 3D control points.
     */
    void SetGlobalControlPoints(const std::vector<Vector3>& points) {
        m_globalPoints = points;
    }

    /**
     * @brief Adds a new patch to the surface by specifying its control point indices.
     *
     * @param indices A 4x4 array of integers, where each integer is an index
     *                into the Global Control Points vector.
     */
    void AddPatch(const int indices[4][4]) {
        PatchDefinition def;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                def.indices[i][j] = indices[i][j];
            }
        }
        m_patches.push_back(def);
    }

    /**
     * @brief Clears all patch definitions.
     */
    void ClearPatches() {
        m_patches.clear();
    }

    /**
     * @brief Generates the combined mesh for all patches.
     *
     * This method iterates through every defined patch, calculates its local mesh
     * using the base CubicBezierPatch class, and merges the results into a single
     * vertex and index buffer suitable for OpenGL rendering.
     *
     * @param steps Tessellation level (e.g., 20).
     * @param outVertices Output vector for combined vertices.
     * @param outIndices Output vector for combined triangle indices.
     */
    void GenerateSurface(int steps, std::vector<Vector3>& outVertices, std::vector<unsigned int>& outIndices) {
        outVertices.clear();
        outIndices.clear();

        if (m_patches.empty() || m_globalPoints.empty()) return;

        // Create a worker instance of the base class
        CubicBezierPatch workerPatch;

        // Offset tracker for indices merging
        // When merging mesh B into mesh A, mesh B's indices need to be shifted
        // by the number of vertices already present in A.
        unsigned int indexOffset = 0;

        for (const auto& patchDef : m_patches) {
            // 1. Extract the 16 control points for the current patch
            Vector3 localControlPoints[4][4];
            bool validPatch = true;

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    int idx = patchDef.indices[i][j];
                    // Safety check
                    if (idx < 0 || idx >= static_cast<int>(m_globalPoints.size())) {
                        validPatch = false;
                        break;
                    }
                    localControlPoints[i][j] = m_globalPoints[idx];
                }
            }

            if (!validPatch) continue;

            // 2. Configure the worker patch
            workerPatch.SetControlPoints(localControlPoints);

            // 3. Generate local mesh data
            std::vector<Vector3> localVertices;
            std::vector<unsigned int> localIndices;
            workerPatch.GenerateSurface(steps, localVertices, localIndices);

            // 4. Merge into output buffers

            // Append vertices
            outVertices.insert(outVertices.end(), localVertices.begin(), localVertices.end());

            // Append indices with offset correction
            for (unsigned int idx : localIndices) {
                outIndices.push_back(idx + indexOffset);
            }

            // Update offset for the next patch
            indexOffset += static_cast<unsigned int>(localVertices.size());
        }
    }

    /**
     * @brief Generates lines representing the control grids of all patches.
     * Useful for debugging to see the skeleton of the surface.
     *
     * @param outLineVertices Output vector for line segments (pairs of points).
     */
    void GenerateControlGridLines(std::vector<Vector3>& outLineVertices) {
        outLineVertices.clear();

        if (m_patches.empty() || m_globalPoints.empty()) return;

        CubicBezierPatch workerPatch;

        for (const auto& patchDef : m_patches) {
            Vector3 localControlPoints[4][4];
            // Extract points (simplified check)
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    int idx = patchDef.indices[i][j];
                    if (idx >= 0 && idx < m_globalPoints.size()) {
                        localControlPoints[i][j] = m_globalPoints[idx];
                    }
                }
            }

            workerPatch.SetControlPoints(localControlPoints);

            // Append local grid lines
            std::vector<Vector3> localLines;
            workerPatch.GetControlGridLines(localLines);
            outLineVertices.insert(outLineVertices.end(), localLines.begin(), localLines.end());
        }
    }

private:
    std::vector<Vector3> m_globalPoints;      // The pool of all control points
    std::vector<PatchDefinition> m_patches;   // Definitions of individual patches
};
