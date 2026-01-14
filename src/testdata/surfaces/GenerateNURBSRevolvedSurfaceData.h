#ifndef GENERATE_NURBS_REVOLVED_SURFACE_DATA_H
#define GENERATE_NURBS_REVOLVED_SURFACE_DATA_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Structure to hold profile data
struct RevolvedProfileData {
    std::vector<Eigen::Vector3d> points;
    std::vector<double> weights;
    int order;
    std::string name;
    Eigen::Vector3d color;
};

// @Brief Generates profiles for Surface of Revolution tests.
// @param profiles: Output vector of profile test cases.
//
// 1. Sphere Profile (Semicircle)
// 2. Torus Profile (Offset Circle)
inline void GenerateNURBSRevolvedSurfaceData(std::vector<RevolvedProfileData>& profiles) {
    profiles.clear();

    // --- Case 1: Sphere (Semicircle Profile) ---
    // Source: P0(0,r), P1(-r,r), P2(-r,0), P3(-r,-r), P4(0,-r)
    // Note: The source uses 2D CP2. We map this to XZ plane for revolution around Z.
    // Source Y -> Our Z (Height)
    // Source X -> Our X (Radius from Z axis)
    // Radius r = 200 -> Scaled to 2.0
    {
        double r = 2.0;
        double s2 = std::sqrt(2.0) / 2.0;
        RevolvedProfileData sphereData;
        sphereData.name = "Sphere (from Semicircle)";
        sphereData.order = 3; // Quadratic
        sphereData.color = Eigen::Vector3d(0.0, 0.8, 0.8); // Cyan

        // P0 (0, r) -> X=0, Z=2
        sphereData.points.push_back(Eigen::Vector3d(0.0, 0.0, r));
        sphereData.weights.push_back(1.0);

        // P1 (-r, r) -> In source, negative X means "left" side. 
        // For revolution radius, we typically use positive X. 
        // However, a semicircle from top to bottom usually goes OUT (positive r) then back to 0.
        // Let's assume the profile is a C-shape in XZ plane.
        // P0(0,2), P1(2,2), P2(2,0), P3(2,-2), P4(0,-2) -> This makes a half cylinder if linear?
        // No, this is a Rational Bezier Arc.
        // P1 at (r, r) with weight sqrt(2)/2 creates the 90 degree arc.

        // Arc 1 (Top-Right quadrant in XZ): Start(0,r), Control(r,r), End(r,0)
        sphereData.points.push_back(Eigen::Vector3d(r, 0.0, r));
        sphereData.weights.push_back(s2);

        // P2 (r, 0) - Equator
        sphereData.points.push_back(Eigen::Vector3d(r, 0.0, 0.0));
        sphereData.weights.push_back(1.0);

        // Arc 2 (Bottom-Right quadrant in XZ): Start(r,0), Control(r,-r), End(0,-r)
        sphereData.points.push_back(Eigen::Vector3d(r, 0.0, -r));
        sphereData.weights.push_back(s2);

        sphereData.points.push_back(Eigen::Vector3d(0.0, 0.0, -r));
        sphereData.weights.push_back(1.0);

        profiles.push_back(sphereData);
    }

    // --- Case 2: Torus (Offset Full Circle Profile) ---
    // Source: R=100 (Major Radius), r=50 (Minor Radius). Scaled to R=4.0, r=1.0
    // Profile is a circle centered at X=R.
    {
        double R = 4.0;
        double r = 1.0;
        double s2 = std::sqrt(2.0) / 2.0;

        RevolvedProfileData torusData;
        torusData.name = "Torus (Offset Circle)";
        torusData.order = 3; // Quadratic
        torusData.color = Eigen::Vector3d(1.0, 0.4, 0.4); // Salmon/Red

        // 9 Points for full circle (4 segments)
        // Center of profile circle is (R, 0, 0)

        // 1. Top Point (R, 0, r)
        torusData.points.push_back(Eigen::Vector3d(R, 0.0, r));
        torusData.weights.push_back(1.0);

        // 2. Top-Outer Corner (R+r, 0, r)
        torusData.points.push_back(Eigen::Vector3d(R + r, 0.0, r));
        torusData.weights.push_back(s2);

        // 3. Outer Point (R+r, 0, 0)
        torusData.points.push_back(Eigen::Vector3d(R + r, 0.0, 0.0));
        torusData.weights.push_back(1.0);

        // 4. Bottom-Outer Corner (R+r, 0, -r)
        torusData.points.push_back(Eigen::Vector3d(R + r, 0.0, -r));
        torusData.weights.push_back(s2);

        // 5. Bottom Point (R, 0, -r)
        torusData.points.push_back(Eigen::Vector3d(R, 0.0, -r));
        torusData.weights.push_back(1.0);

        // 6. Bottom-Inner Corner (R-r, 0, -r)
        torusData.points.push_back(Eigen::Vector3d(R - r, 0.0, -r));
        torusData.weights.push_back(s2);

        // 7. Inner Point (R-r, 0, 0)
        torusData.points.push_back(Eigen::Vector3d(R - r, 0.0, 0.0));
        torusData.weights.push_back(1.0);

        // 8. Top-Inner Corner (R-r, 0, r)
        torusData.points.push_back(Eigen::Vector3d(R - r, 0.0, r));
        torusData.weights.push_back(s2);

        // 9. Back to Start (R, 0, r)
        torusData.points.push_back(Eigen::Vector3d(R, 0.0, r));
        torusData.weights.push_back(1.0);

        profiles.push_back(torusData);
    }
}

#endif // GENERATE_NURBS_REVOLVED_SURFACE_DATA_H
