#ifndef GENERATE_NURBS_CIRCLE_APPROXIMATOR_DATA_H
#define GENERATE_NURBS_CIRCLE_APPROXIMATOR_DATA_H

#include <vector>
#include <string>
#include <cmath>
#include <glm/glm.hpp> // Used for color definition

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline double DegToRad(double deg) {
    return deg * M_PI / 180.0;
}

// Structure to hold test parameters
struct CircleArcTestCase {
    double radius;
    double startAngleRad;
    double sweepAngleRad;
    std::string name;
    glm::vec3 color;
};

// @Brief Generates test parameters for NURBS Circle Approximation.
// @param cases: Output vector of test cases.
//
// Replicates the 4 examples from the source code:
// 1. Small Arc (80 deg)
// 2. Medium Arc (150 deg)
// 3. Large Arc (240 deg)
// 4. Near-Circle Arc (300 deg)
inline void GenerateNURBSCircleApproximatorData(std::vector<CircleArcTestCase>& cases) {
    cases.clear();

    // Case 1: 80 degrees (< 90)
    // Source: r=200, Theta=80, Phi=50
    cases.push_back({
        2.0,                // Radius (Scaled 200 -> 2.0)
        DegToRad(50.0),     // Start Angle
        DegToRad(80.0),     // Sweep Angle
        "Arc 80 deg",       // Name
        glm::vec3(1.0f, 0.0f, 0.0f) // Red
        });

    // Case 2: 150 degrees (> 90, likely 2 segments)
    // Source: r=200, Theta=150, Phi=15
    // Increased radius slightly to visualize concentric-ish
    cases.push_back({
        3.0,
        DegToRad(15.0),
        DegToRad(150.0),
        "Arc 150 deg",
        glm::vec3(0.0f, 1.0f, 0.0f) // Green
        });

    // Case 3: 240 degrees (Multiple segments)
    // Source: r=200, Theta=240, Phi=30
    cases.push_back({
        4.0,
        DegToRad(30.0),
        DegToRad(240.0),
        "Arc 240 deg",
        glm::vec3(0.0f, 0.5f, 1.0f) // Cyan/Blue
        });

    // Case 4: 300 degrees (Almost full circle)
    // Source: r=200, Theta=300, Phi=15
    cases.push_back({
        5.0,
        DegToRad(15.0),
        DegToRad(300.0),
        "Arc 300 deg",
        glm::vec3(1.0f, 0.0f, 1.0f) // Magenta
        });
}

#endif // GENERATE_NURBS_CIRCLE_APPROXIMATOR_DATA_H
