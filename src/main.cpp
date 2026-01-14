#include "Application.h"
#include <iostream>
#include <vector>
#include <functional>
#include <string>

// --- ImGui Libraries ---
#include <imgui.h>

// --- Geometry Class Headers ---
#include "geometry/curves/CubicSpline.h"
#include "geometry/curves/CardinalCurve.h"
#include "geometry/curves/CubicBezierCurve.h"
#include "geometry/curves/CompositeBezierCurve.h"
#include "geometry/curves/RationalQuadraticBezierCurve.h"
#include "geometry/curves/DegreeElevatedRationalBezierCurve.h"
#include "geometry/curves/UniformBSplineCurve.h"
#include "geometry/curves/QuasiUniformBSplineCurve.h"
#include "geometry/curves/PiecewiseBezierCurve.h"
#include "geometry/curves/NonUniformBSplineCurve.h"
#include "geometry/curves/BSplineKnotInserter.h"
#include "geometry/curves/NURBSCurve.h"
#include "geometry/curves/NURBSCircleApproximator.h"

#include "geometry/surfaces/CubicBezierPatch.h"
#include "geometry/surfaces/CompositeBezierSurface.h"
#include "geometry/surfaces/RationalBiquadraticBezierPatch.h"
#include "geometry/surfaces/RationalBezierPatch2x1.h"
#include "geometry/surfaces/CubicUniformBSplinePatch.h"
#include "geometry/surfaces/NonUniformBSplineSurface.h"
#include "geometry/surfaces/NURBSSurface.h"
#include "geometry/surfaces/NURBSRevolvedSurface.h"

// --- Data Generation Headers ---
#include "testdata/curves/GenerateCubicSplineData.h"
#include "testdata/curves/GenerateCardinalCurveData.h"
#include "testdata/curves/GenerateBezierCurveData.h"
#include "testdata/curves/GenerateCompositeBezierCurveData.h"
#include "testdata/curves/GenerateRationalBezierCurveData.h"
#include "testdata/curves/GenerateDegreeElevatedRationalBezierCurveData.h"
#include "testdata/curves/GenerateUniformBSplineCurveData.h"
#include "testdata/curves/GenerateQuasiUniformBSplineCurveData.h"
#include "testdata/curves/GeneratePiecewiseBezierCurveData.h"
#include "testdata/curves/GenerateNonUniformBSplineCurveData.h"
#include "testdata/curves/GenerateBSplineKnotInserterData.h"
#include "testdata/curves/GenerateNURBSCurveData.h"
#include "testdata/curves/GenerateNURBSCircleApproximatorData.h"

#include "testdata/surfaces/GenerateCubicBezierData.h"
#include "testdata/surfaces/GenerateCompositeBezierSurfaceData.h"
#include "testdata/surfaces/GenerateRationalBiquadraticBezierPatchData.h"
#include "testdata/surfaces/GenerateRationalBezierPatch2x1Data.h"
#include "testdata/surfaces/GenerateCubicUniformBSplinePatchData.h"
#include "testdata/surfaces/GenerateNonUniformBSplineSurfaceData.h"
#include "testdata/surfaces/GenerateNURBSSurfaceData.h"
#include "testdata/surfaces/GenerateNURBSRevolvedSurfaceData.h"

struct TestCase {
    std::string name;
    std::function<void(Application&)> run;
};

void ResetScene(Application& app) {
    app.geometryRenderer.clear();
    app.geometryRenderer.addAxes(100.0f);
}

int main() {
    Application app;
    std::vector<TestCase> tests;

    // --- 01. Cubic Spline ---
    tests.push_back({ "01. Cubic Spline", [](Application& app) {
        std::vector<Eigen::Vector3d> points;
        GenerateCubicSplineData(points);
        CubicSpline spline;
        spline.SetControlPoints(points, 1.0, 1.0);
        std::vector<Vector3> drawPts;
        spline.GenerateCurve(0.5f, drawPts);

        // Draw Curve
        GeometryData d(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(drawPts, glm::vec3(0.2f, 0.9f, 1.0f), d);
        app.geometryRenderer.addGeometry(d);

        // [NEW] Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(points, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 02. Cardinal Curve ---
    tests.push_back({ "02. Cardinal Curve", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GenerateCardinalCurveData(pts);
        CardinalCurve card;
        card.SetControlPoints(pts);
        card.SetTension(0.5);
        std::vector<Vector3> drawPts;
        card.GenerateCurve(0.1f, drawPts);

        GeometryData d(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(drawPts, glm::vec3(1.0f, 0.2f, 0.8f), d);
        app.geometryRenderer.addGeometry(d);

        // [NEW] Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 03. Cubic Bezier Curve ---
    tests.push_back({ "03. Cubic Bezier Curve", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GenerateBezierCurveData(pts);
        CubicBezierCurve bezier;
        bezier.SetControlPoints(pts);
        std::vector<Vector3> drawPts;
        bezier.GenerateCurve(0.02f, drawPts);

        GeometryData d(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(drawPts, glm::vec3(1.0f, 0.5f, 0.5f), d);
        app.geometryRenderer.addGeometry(d);

        // [EXISTING] Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 04. Composite Bezier Curve ---
    tests.push_back({ "04. Composite Bezier Curve", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GenerateCompositeBezierCurveData(pts);
        CompositeBezierCurve comp;
        comp.SetControlPoints(pts);
        std::vector<Vector3> drawPts;
        comp.GenerateCurve(0.05f, drawPts);

        GeometryData d(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(drawPts, glm::vec3(0.6f, 0.2f, 0.8f), d);
        app.geometryRenderer.addGeometry(d);

        // [EXISTING/NEW] Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 05. Cubic Bezier Surface (Skipped Curve Logic) ---
    tests.push_back({ "05. Cubic Bezier Surface", [](Application& app) {
        Eigen::Vector3d pts[4][4];
        GenerateCubicBezierData(pts);
        CubicBezierPatch patch;
        patch.SetControlPoints(pts);
        std::vector<Vector3> v;
        std::vector<unsigned int> i;
        patch.GenerateSurface(20, v, i);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(1.0f, 0.75f, 0.3f), d);
        app.geometryRenderer.addGeometry(d);
        // Note: Surfaces usually draw control GRIDS, which requires a grid-to-line converter, already handled in other tests.
    } });

    // --- 06. Composite Bezier Surface (Skipped) ---
    tests.push_back({ "06. Composite Bezier Surface", [](Application& app) {
        std::vector<Vector3> pts;
        std::vector<RawPatchIndices> patches;
        GenerateCompositeBezierSurfaceData(pts, patches);
        CompositeBezierSurface surf;
        surf.SetGlobalControlPoints(pts);
        for (const auto& p : patches) surf.AddPatch(p.indices);
        std::vector<Vector3> v;
        std::vector<unsigned int> i;
        surf.GenerateSurface(15, v, i);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(1.0f, 0.75f, 0.3f), d);
        app.geometryRenderer.addGeometry(d);
    } });

    // --- 07. Rational Quadratic Bezier ---
    tests.push_back({ "07. Rational Quadratic Bezier", [](Application& app) {
        std::vector<Eigen::Vector3d> basePts;
        GenerateRationalBezierCurveData(basePts);
        struct C { double w; glm::vec3 c; float z; };
        std::vector<C> cases = {
            {2.0, {0.6f,0.f,0.8f}, 0.f},
            {1.0, {1.f,0.5f,0.f}, 1.f},
            {0.5, {1.f,1.f,0.f}, 3.f},
            {-0.5, {0.0f,1.f,1.f}, 4.f}
        };
        for (const auto& t : cases) {
            std::vector<Eigen::Vector3d> pts = basePts;
            for (auto& p : pts) p.z() += t.z;
            RationalQuadraticBezierCurve c;
            c.SetControlPoints(pts);
            c.SetMiddleWeight(t.w);
            std::vector<Vector3> out;
            c.GenerateCurve(0.02f, out);

            GeometryData d(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(out, t.c, d);
            app.geometryRenderer.addGeometry(d);

            // [EXISTING/NEW] Draw Control Polygon
            GeometryData poly(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
            app.geometryRenderer.addGeometry(poly);
        }
    } });

    // --- 08. Degree Elevated Rational Bezier ---
    tests.push_back({ "08. Degree Elevated Rational Bezier", [](Application& app) {
        std::vector<RationalSegmentData> segs;
        GenerateDegreeElevatedRationalBezierCurveData(segs);
        std::vector<glm::vec3> colors = { {0.6f,0.f,0.8f}, {1.f,0.5f,0.f}, {1.f,1.f,1.f}, {1.f,1.f,0.f} };
        int idx = 0;
        for (const auto& s : segs) {
            DegreeElevatedRationalBezierCurve c;
            std::vector<Vector3> pts = { s.P0, s.P1, s.P2 };
            c.SetControlPoints(pts);
            c.SetMiddleWeight(s.w1);
            std::vector<Vector3> out;
            c.GenerateCurve(0.02f, out);

            GeometryData d(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(out, colors[idx % 4], d);
            app.geometryRenderer.addGeometry(d);
            idx++;

            // [EXISTING/NEW] Draw Control Polygon
            GeometryData poly(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
            app.geometryRenderer.addGeometry(poly);
        }
    } });

    // --- 09. Rational Biquadratic Patch (Skipped) ---
    tests.push_back({ "09. Rational Biquadratic Patch", [](Application& app) {
        Vector3 p[3][3];
        double w[3][3];
        GenerateRationalBiquadraticBezierPatchData(p, w);
        RationalBiquadraticBezierPatch patch;
        patch.SetControlData(p, w);
        std::vector<Vector3> v;
        std::vector<unsigned int> i;
        patch.GenerateSurface(20, v, i);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(0.0f, 1.0f, 1.0f), d);
        app.geometryRenderer.addGeometry(d);
    } });

    // --- 10. Rational Bezier Patch 2x1 (Skipped) ---
    tests.push_back({ "10. Rational Bezier Patch 2x1", [](Application& app) {
        Vector3 p[3][2]; double w[3][2];
        GenerateCylinderPatchData(p, w);
        RationalBezierPatch2x1 cyl;
        cyl.SetControlData(p, w);
        std::vector<Vector3> v; std::vector<unsigned int> i;
        cyl.GenerateSurface(15, v, i);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(1.0f, 0.5f, 0.0f), d);
        app.geometryRenderer.addGeometry(d);

        GenerateConePatchData(p, w);
        for (int x = 0; x < 3; ++x) for (int y = 0; y < 2; ++y) p[x][y].x() += 5.0;
        RationalBezierPatch2x1 cone;
        cone.SetControlData(p, w);
        cone.GenerateSurface(15, v, i);
        GeometryData d2(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(0.6f, 0.0f, 0.8f), d2);
        app.geometryRenderer.addGeometry(d2);
    } });

    // --- 11. Uniform B-Spline Curve ---
    tests.push_back({ "11. Uniform B-Spline Curve", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GenerateUniformBSplineCurveData(pts);
        UniformBSplineCurve spline(4);
        spline.SetControlPoints(pts);
        std::vector<Vector3> out;
        spline.GenerateCurve(0.05, out);
        if (!out.empty()) {
            GeometryData d(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(out, glm::vec3(0.2f, 1.0f, 0.2f), d);
            app.geometryRenderer.addGeometry(d);
        }
        // Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly); // Changed to White
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 12. Quasi-Uniform B-Spline ---
    tests.push_back({ "12. Quasi-Uniform B-Spline", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GenerateQuasiUniformBSplineCurveData(pts);
        QuasiUniformBSplineCurve spline(4);
        spline.SetControlPoints(pts);
        std::vector<Vector3> out;
        spline.GenerateCurve(0.02, out);
        if (!out.empty()) {
            GeometryData d(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(out, glm::vec3(0.0f, 1.0f, 1.0f), d);
            app.geometryRenderer.addGeometry(d);
        }
        // Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 13. Piecewise Bezier Curve ---
    tests.push_back({ "13. Piecewise Bezier Curve", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GeneratePiecewiseBezierCurveData(pts);
        PiecewiseBezierCurve spline(4);
        spline.SetControlPoints(pts);
        std::vector<Vector3> out;
        spline.GenerateCurve(0.01, out);
        if (!out.empty()) {
            GeometryData d(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(out, glm::vec3(1.0f, 0.0f, 1.0f), d);
            app.geometryRenderer.addGeometry(d);
        }
        // [NEW] Draw Control Polygon
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 14. Non-Uniform B-Spline ---
    tests.push_back({ "14. Non-Uniform B-Spline", [](Application& app) {
        std::vector<Eigen::Vector3d> pts;
        GenerateNonUniformBSplineCurveData(pts);
        NonUniformBSplineCurve spline(4);
        spline.SetControlPoints(pts);
        std::vector<Vector3> out;
        spline.GenerateCurve(0.01, out);
        if (!out.empty()) {
            GeometryData d(GeometryType::CURVE);
            app.geometryRenderer.convertCurveToGeometry(out, glm::vec3(1.0f, 0.27f, 0.0f), d);
            app.geometryRenderer.addGeometry(d);
        }
        // [EXISTING] Control Polygon - Updated color to White
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(pts, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 15. Knot Insertion ---
    // Demonstrates Boehm's Algorithm for inserting knots into a B-Spline.
    // KEY CONCEPT: The curve shape MUST remain identical before and after insertion.
    // The control polygon changes (gets closer to the curve), but the curve geometry does not.
    tests.push_back({ "15. Knot Insertion", [](Application& app) {
        std::vector<Eigen::Vector3d> initPoints;
        std::vector<double> initKnots;

        // 1. Generate Initial Data
        GenerateBSplineKnotInserterData(initPoints, initKnots);

        // 2. Perform Knot Insertion
        BSplineKnotInserter inserter(4, initPoints, initKnots);
        inserter.InsertKnot(0.25);
        inserter.InsertKnot(0.75);

        // Get refined data
        std::vector<Vector3> refinedPoints = inserter.GetNewControlPoints();
        std::vector<double> refinedKnots = inserter.GetNewKnots();

        // 3. Visualize Control Polygons

        // Original Polygon (Grey)
        GeometryData oldPoly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(initPoints, glm::vec3(0.5f), oldPoly);
        app.geometryRenderer.addGeometry(oldPoly);

        // Refined Polygon (cyan) - Should be closer to the curve
        GeometryData newPoly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(refinedPoints, glm::vec3(0.2f, 0.9f, 1.0f), newPoly);
        app.geometryRenderer.addGeometry(newPoly);

        // 4. Visualize The Curve (Using NonUniformBSplineCurve calculator)
        // We use the REFINED data to draw the curve.
        // (Mathematically, drawing with initPoints would yield the exact same curve)

        NonUniformBSplineCurve splineCalculator(4);
        splineCalculator.SetControlPoints(refinedPoints);

        std::vector<Vector3> curvePts;
        splineCalculator.GenerateCurve(0.01, curvePts); // High resolution

        GeometryData curveGeom(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(curvePts, glm::vec3(1.0f, 0.27f, 0.0f), curveGeom); // Orange Curve
        app.geometryRenderer.addGeometry(curveGeom);

    } });


    // --- 16. Bicubic Uniform B-Spline Patch (Skipped) ---
    tests.push_back({ "16. Bicubic Uniform B-Spline Patch", [](Application& app) {
        Vector3 pts[4][4];
        GenerateCubicUniformBSplinePatchData(pts);
        CubicUniformBSplinePatch patch;
        patch.SetControlPoints(pts);
        std::vector<Vector3> v;
        std::vector<unsigned int> i;
        patch.GenerateSurface(20, v, i);
        std::vector<Vector3> net;
        patch.GetControlGridLines(net);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(0.0f, 0.5f, 0.5f), d);
        app.geometryRenderer.addGeometry(d);
        GeometryData n(GeometryType::GRID);
        app.geometryRenderer.convertGridToGeometry(net, glm::vec3(1.0f, 1.0f, 0.0f), n);
        app.geometryRenderer.addGeometry(n);
    } });

    // --- 17. Non-Uniform B-Spline Surface (Skipped) ---
    tests.push_back({ "17. Non-Uniform B-Spline Surface", [](Application& app) {
        std::vector<std::vector<Vector3>> grid;
        GenerateNonUniformBSplineSurfaceData(grid);
        NonUniformBSplineSurface surf(4, 3);
        surf.SetControlGrid(grid);
        std::vector<Vector3> v;
        std::vector<unsigned int> i;
        surf.GenerateSurface(20, v, i);
        std::vector<Vector3> net;
        surf.GetControlGridLines(net);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(0.4f, 0.35f, 0.8f), d);
        app.geometryRenderer.addGeometry(d);
        GeometryData n(GeometryType::GRID);
        app.geometryRenderer.convertGridToGeometry(net, glm::vec3(1.0f), n);
        app.geometryRenderer.addGeometry(n);
    } });

    // --- 18. NURBS Curve ---
    tests.push_back({ "18. NURBS Curve", [](Application& app) {
        std::vector<Vector3> p;
        std::vector<double> w;
        GenerateNURBSCurveData(p, w);
        NURBSCurve nurbs(4);
        nurbs.SetControlData(p, w);
        std::vector<Vector3> out;
        nurbs.GenerateCurve(0.01, out);

        GeometryData d(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(out, glm::vec3(0.0f, 0.0f, 1.0f), d);
        app.geometryRenderer.addGeometry(d);

        // [EXISTING] Control Polygon - Updated color to White
        GeometryData poly(GeometryType::CURVE);
        app.geometryRenderer.convertCurveToGeometry(p, glm::vec3(1.0f, 1.0f, 1.0f), poly);
        app.geometryRenderer.addGeometry(poly);
    } });

    // --- 19. NURBS Circle ---
    tests.push_back({ "19. NURBS Circle", [](Application& app) {
        std::vector<CircleArcTestCase> cases;
        GenerateNURBSCircleApproximatorData(cases);
        NURBSCircleApproximator approx;
        for (const auto& c : cases) {
            approx.SetupArc(c.radius, c.startAngleRad, c.sweepAngleRad);
            std::vector<Vector3> out;
            approx.GenerateCurve(0.01, out);
            const std::vector<Vector3>& arcCtrlPoints = approx.GetControlPoints();
            if (!out.empty()) {
                GeometryData d(GeometryType::CURVE);
                app.geometryRenderer.convertCurveToGeometry(out, c.color, d);
                app.geometryRenderer.addGeometry(d);

                // [EXISTING] Control Polygon - Updated color to White
                GeometryData poly(GeometryType::CURVE);
                app.geometryRenderer.convertCurveToGeometry(arcCtrlPoints, glm::vec3(1.0f, 1.0f, 1.0f), poly);
                app.geometryRenderer.addGeometry(poly);
            }
        }
    } });

    // --- 20. NURBS Surface (Skipped) ---
    tests.push_back({ "20. NURBS Surface (High Weight)", [](Application& app) {
        std::vector<std::vector<Vector3>> p;
        std::vector<std::vector<double>> w;
        GenerateNURBSSurfaceData(p, w);
        NURBSSurface surf(4, 3);
        surf.SetControlData(p, w);
        std::vector<Vector3> v;
        std::vector<unsigned int> i;
        surf.GenerateSurface(25, v, i);
        std::vector<Vector3> net;
        surf.GetControlGridLines(net);
        GeometryData d(GeometryType::SURFACE);
        app.geometryRenderer.convertSurfaceToGeometry(v, i, glm::vec3(1.0f, 0.84f, 0.0f), d);
        app.geometryRenderer.addGeometry(d);
        GeometryData n(GeometryType::GRID);
        app.geometryRenderer.convertGridToGeometry(net, glm::vec3(1.0f), n);
        app.geometryRenderer.addGeometry(n);
    } });

    // --- 21. NURBS Revolved Surface (Skipped) ---
    tests.push_back({ "21. NURBS Revolved Surface", [](Application& app) {
        std::vector<RevolvedProfileData> profs;
        GenerateNURBSRevolvedSurfaceData(profs);
        NURBSRevolvedSurface rev;
        for (const auto& p : profs) {
            rev.SetupProfile(p.points, p.weights, p.order);
            std::vector<Vector3> v;
            std::vector<unsigned int> i;
            rev.GenerateSurface(20, v, i);
            std::vector<Vector3> net;
            rev.GetControlGridLines(net);
            GeometryData d(GeometryType::SURFACE);
            glm::vec3 c(p.color.x(), p.color.y(), p.color.z());
            app.geometryRenderer.convertSurfaceToGeometry(v, i, c, d);
            app.geometryRenderer.addGeometry(d);
            GeometryData n(GeometryType::GRID);
            app.geometryRenderer.convertGridToGeometry(net, glm::vec3(0.3f), n);
            app.geometryRenderer.addGeometry(n);
        }
    } });

    static int selectedTest = -1;
    app.setUICallback([&]() {
        ImGui::Begin("Computational Geometry Tests");
        if (ImGui::Button("Clear Scene")) {
            ResetScene(app);
            selectedTest = -1;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Removes all geometry curves and surfaces from the view.");
        }
        ImGui::Separator();
        const char* currentName = (selectedTest >= 0) ? tests[selectedTest].name.c_str() : "Select a test...";
        if (ImGui::BeginCombo("Test Cases", currentName)) {
            for (int n = 0; n < tests.size(); n++) {
                bool isSelected = (selectedTest == n);
                if (ImGui::Selectable(tests[n].name.c_str(), isSelected)) {
                    selectedTest = n;
                    ResetScene(app);
                    tests[n].run(app);
                }
                if (isSelected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::End();
        });

    std::cout << "Application initialized. Starting render loop." << std::endl;
    return app.run();
}
