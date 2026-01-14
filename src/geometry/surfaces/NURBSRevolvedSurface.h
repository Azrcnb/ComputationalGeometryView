#ifndef NURBS_REVOLVED_SURFACE_H
#define NURBS_REVOLVED_SURFACE_H

#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include "NURBSSurface.h" // Re-use the general surface evaluator

// Using Eigen's Vector3d
using Vector3 = Eigen::Vector3d;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Generates a Surface of Revolution using NURBS.
 *
 * This class takes a 2D profile curve (Generatrix) defined by control points and weights,
 * and revolves it around the Z-axis to create a 3D NURBS surface.
 *
 * It automatically constructs the necessary control grid and weights for the
 * rotational direction (U direction) using the standard 9-point circle representation.
 */
class NURBSRevolvedSurface {
public:
    /**
     * @brief Constructor.
     */
    NURBSRevolvedSurface() = default;
    ~NURBSRevolvedSurface() = default;

    /**
     * @brief Setup the surface by revolving a profile curve.
     *
     * @param profilePoints Control points of the profile curve (in XZ plane, revolving around Z).
     *                      Ideally, these should have y=0.
     * @param profileWeights Weights of the profile curve.
     * @param profileOrder Order of the profile curve (e.g., 3 for Quadratic, 4 for Cubic).
     */
    void SetupProfile(const std::vector<Vector3>& profilePoints,
        const std::vector<double>& profileWeights,
        int profileOrder) {

        if (profilePoints.size() != profileWeights.size() || profilePoints.empty()) {
            std::cerr << "Error: Invalid profile data." << std::endl;
            return;
        }

        // 1. Define the Circle (U direction)
        // Standard full circle NURBS representation:
        // Order: 3 (Quadratic)
        // Control Points: 9 (forming a square bounding box logic)
        // Weights: 1, sqrt(2)/2, 1, sqrt(2)/2, ...

        int circleOrder = 3;
        int numCirclePoints = 9;

        // Standard weights for a unit circle
        double wCircle[9] = {
            1.0,
            std::sqrt(2.0) / 2.0,
            1.0,
            std::sqrt(2.0) / 2.0,
            1.0,
            std::sqrt(2.0) / 2.0,
            1.0,
            std::sqrt(2.0) / 2.0,
            1.0
        };

        // Standard cosines/sines for the 9 points (0, 45, 90, 135, ...)
        // Used to rotate the profile point
        double angles[9] = { 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 360.0 };

        // 2. Generate Tensor Product Grid
        // Grid size: [9] x [numProfilePoints]
        int n = numCirclePoints;
        int m = static_cast<int>(profilePoints.size());

        std::vector<std::vector<Vector3>> surfPoints(n, std::vector<Vector3>(m));
        std::vector<std::vector<double>> surfWeights(n, std::vector<double>(m));

        for (int j = 0; j < m; ++j) {
            Vector3 P_profile = profilePoints[j]; // The point to revolve
            double w_profile = profileWeights[j];

            // Radius of this specific point from Z axis
            double r = P_profile.x();
            double z = P_profile.z();

            // Revolve P_profile to create the 9 U-points
            // The circle logic: x = r*cos(theta), y = r*sin(theta)
            // But for the corners of the square control polygon (angles 45, 135...),
            // the radius is actually r / cos(45) = r * sqrt(2).
            // HOWEVER, the weights handle the projection!
            // So we simply place the control points on the square/octagon bounding box.

            // Standard Unit Circle Control Points (in XY plane):
            // (1,0), (1,1), (0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1), (1,0)
            // We scale these by 'r' and set 'z'.

            Vector3 circleUnitPoints[9] = {
                Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(0, 1, 0),
                Vector3(-1, 1, 0), Vector3(-1, 0, 0), Vector3(-1, -1, 0),
                Vector3(0, -1, 0), Vector3(1, -1, 0), Vector3(1, 0, 0)
            };

            for (int i = 0; i < n; ++i) {
                Vector3 unitP = circleUnitPoints[i];

                // Scale by the profile point's radius 'r' and position at 'z'
                // Note: If r is negative (e.g. left side of profile), math still holds.
                Vector3 finalP(unitP.x() * r, unitP.y() * r, z);

                surfPoints[i][j] = finalP;
                surfWeights[i][j] = wCircle[i] * w_profile;
            }
        }

        // 3. Configure the internal NURBSSurface
        // U direction: Circle (Quadratic, Order 3)
        // V direction: Profile (User defined Order)
        m_surfaceEvaluator = std::make_unique<NURBSSurface>(circleOrder, profileOrder);
        m_surfaceEvaluator->SetControlData(surfPoints, surfWeights);
    }

    /**
     * @brief Generates mesh data.
     * Delegates to the internal NURBSSurface.
     */
    void GenerateSurface(int steps, std::vector<Vector3>& vertices, std::vector<unsigned int>& indices) {
        if (m_surfaceEvaluator) {
            m_surfaceEvaluator->GenerateSurface(steps, vertices, indices);
        }
    }

    /**
     * @brief Generates control grid lines.
     */
    void GetControlGridLines(std::vector<Vector3>& lineVertices) {
        if (m_surfaceEvaluator) {
            m_surfaceEvaluator->GetControlGridLines(lineVertices);
        }
    }

private:
    std::unique_ptr<NURBSSurface> m_surfaceEvaluator;
};

#endif // NURBS_REVOLVED_SURFACE_H
