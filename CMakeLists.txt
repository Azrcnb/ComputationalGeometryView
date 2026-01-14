# CMakeList.txt: ComputationalGeometryView  

cmake_minimum_required (VERSION 3.25)

# Please enable hot reloading for the MSVC complier
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("ComputationalGeometryView")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Recommend high warnings
if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(eigen3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

# Add the source code to the executable file of this project
add_executable (ComputationalGeometryView
"src/main.cpp"
 "src/renderer/camera.h" 
 "src/renderer/shader_m.h" 
 "src/geometry/curves/CubicSpline.h" 
 "src/geometry/surfaces/CubicBezierPatch.h"  
 "src/application.h" 
 "src/application.cpp" 
 "src/renderer/GeometryRenderer.h" 
 "src/renderer/GeometryRenderer.cpp"
 "src/testdata/curves/GenerateCubicSplineData.h" 
 "src/testdata/surfaces/GenerateCubicBezierData.h" "src/geometry/curves/CardinalCurve.h" "src/testdata/curves/GenerateCardinalCurveData.h" "src/geometry/curves/CubicBezierCurve.h" "src/testdata/curves/GenerateBezierCurveData.h" "src/geometry/curves/CompositeBezierCurve.h" "src/testdata/curves/GenerateCompositeBezierCurveData.h" "src/geometry/surfaces/CompositeBezierSurface.h" "src/testdata/surfaces/GenerateCompositeBezierSurfaceData.h" "src/geometry/curves/RationalQuadraticBezierCurve.h" "src/testdata/curves/GenerateRationalBezierCurveData.h" "src/geometry/curves/DegreeElevatedRationalBezierCurve.h" "src/testdata/curves/GenerateDegreeElevatedRationalBezierCurveData.h" "src/geometry/surfaces/RationalBiquadraticBezierPatch.h" "src/testdata/surfaces/GenerateRationalBiquadraticBezierPatchData.h" "src/geometry/surfaces/RationalBezierPatch2x1.h" "src/testdata/surfaces/GenerateRationalBezierPatch2x1Data.h" "src/geometry/curves/UniformBSplineCurve.h" "src/testdata/curves/GenerateUniformBSplineCurveData.h" "src/geometry/curves/QuasiUniformBSplineCurve.h" "src/testdata/curves/GenerateQuasiUniformBSplineCurveData.h" "src/geometry/curves/PiecewiseBezierCurve.h" "src/testdata/curves/GeneratePiecewiseBezierCurveData.h" "src/geometry/curves/NonUniformBSplineCurve.h" "src/testdata/curves/GenerateNonUniformBSplineCurveData.h" "src/geometry/curves/BSplineKnotInserter.h" "src/testdata/curves/GenerateBSplineKnotInserterData.h" "src/geometry/surfaces/CubicUniformBSplinePatch.h" "src/testdata/surfaces/GenerateCubicUniformBSplinePatchData.h" "src/geometry/surfaces/NonUniformBSplineSurface.h" "src/testdata/surfaces/GenerateNonUniformBSplineSurfaceData.h" "src/geometry/curves/NURBSCurve.h" "src/testdata/curves/GenerateNURBSCurveData.h" "src/geometry/curves/NURBSCircleApproximator.h" "src/testdata/curves/GenerateNURBSCircleApproximatorData.h" "src/geometry/surfaces/NURBSSurface.h"  "src/testdata/surfaces/GenerateNURBSSurfaceData.h" "src/geometry/surfaces/NURBSRevolvedSurface.h" "src/testdata/surfaces/GenerateNURBSRevolvedSurfaceData.h")

# Copy shader files to the build directory
set(SHADER_SOURCE_DIR "${CMAKE_SOURCE_DIR}/assets/Shader")

add_custom_command(TARGET ComputationalGeometryView POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:ComputationalGeometryView>/Shader"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SHADER_SOURCE_DIR}" "$<TARGET_FILE_DIR:ComputationalGeometryView>/Shader"
    COMMENT "Copying shader files to executable directory..."
)



target_include_directories(ComputationalGeometryView PRIVATE ${CMAKE_SOURCE_DIR}/src)


# On MSVC, prefer main CRT entry point
if(MSVC)
    target_compile_definitions(ComputationalGeometryView PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()


# Include dirs for libraries that need them (glm is header-only)
target_link_libraries(ComputationalGeometryView PRIVATE
    glfw 
    glad::glad 
    glm::glm 
    Eigen3::Eigen
    imgui::imgui
)

