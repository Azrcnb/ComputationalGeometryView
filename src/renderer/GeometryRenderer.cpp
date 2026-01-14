#include "GeometryRenderer.h"
#include <iostream>
#include <vector>
#include <map>
#include <tuple>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

GeometryRenderer::GeometryRenderer() {
}

GeometryRenderer::~GeometryRenderer() {
    clear();
}

void GeometryRenderer::init() {
    shaderProgram = std::make_unique<Shader>("Shader/VertexShader.vs", "Shader/FragmentShader.fs");
}

void GeometryRenderer::clear() {
    geometries.clear();
    if (!vaoHandles.empty()) glDeleteVertexArrays(static_cast<GLsizei>(vaoHandles.size()), vaoHandles.data());
    if (!vboHandles.empty()) glDeleteBuffers(static_cast<GLsizei>(vboHandles.size()), vboHandles.data());
    if (!eboHandles.empty()) glDeleteBuffers(static_cast<GLsizei>(eboHandles.size()), eboHandles.data());
    vaoHandles.clear();
    vboHandles.clear();
    eboHandles.clear();
}

void GeometryRenderer::addGeometry(const GeometryData& geometry) {
    geometries.push_back(geometry);

    GLuint vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, geometry.vertices.size() * sizeof(Vertex), geometry.vertices.data(), GL_STATIC_DRAW);

    if (!geometry.indices.empty()) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, geometry.indices.size() * sizeof(unsigned int), geometry.indices.data(), GL_STATIC_DRAW);
    }

    // Attributes: 0=Pos, 1=Color, 2=Normal
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
    vaoHandles.push_back(vao);
    vboHandles.push_back(vbo);
    eboHandles.push_back(ebo);
}

void GeometryRenderer::addAxes(float length) {
    GeometryData axesData(GeometryType::AXES);
    // X (Red), Y (Green), Z (Blue)
    axesData.vertices.push_back(Vertex(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    axesData.vertices.push_back(Vertex(glm::vec3(length, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    axesData.vertices.push_back(Vertex(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    axesData.vertices.push_back(Vertex(glm::vec3(0.0f, length, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    axesData.vertices.push_back(Vertex(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
    axesData.vertices.push_back(Vertex(glm::vec3(0.0f, 0.0f, length), glm::vec3(0.0f, 0.0f, 1.0f)));
    addGeometry(axesData);
}

void GeometryRenderer::convertCurveToGeometry(const std::vector<Vector3>& curvePoints, const glm::vec3& color, GeometryData& outData) {
    outData.vertices.clear();
    for (const auto& pt : curvePoints) {
        outData.vertices.push_back(Vertex(
            glm::vec3(pt.x(), pt.y(), pt.z()),
            color
        ));
    }
    outData.indices.clear();
}

// Implementation for Grid (Disjoint Lines)
void GeometryRenderer::convertGridToGeometry(const std::vector<Vector3>& gridPoints, const glm::vec3& color, GeometryData& outData) {
    outData.vertices.clear();
    outData.type = GeometryType::GRID;

    for (const auto& pt : gridPoints) {
        outData.vertices.push_back(Vertex(
            glm::vec3(pt.x(), pt.y(), pt.z()),
            color
        ));
    }
    outData.indices.clear();
}

void GeometryRenderer::convertSurfaceToGeometry(const std::vector<Vector3>& meshVertices, const std::vector<unsigned int>& meshIndices, const glm::vec3& color, GeometryData& outData) {
    outData.vertices.clear();

    // 1. Convert Points to Vertices
    for (const auto& p : meshVertices) {
        // Initialize normal to zero
        outData.vertices.push_back(Vertex(glm::vec3(p.x(), p.y(), p.z()), color, glm::vec3(0.0f)));
    }
    outData.indices = meshIndices;

    // 2. Calculate Normals with Spatial Welding
    calculateSmoothNormals(outData.vertices, outData.indices);
}

// --- Modified Function: Spatial Normal Smoothing ---
void GeometryRenderer::calculateSmoothNormals(std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) {
    // 1. Reset all normals to zero
    for (auto& v : vertices) {
        v.normal = glm::vec3(0.0f);
    }

    // 2. Accumulate Face Normals
    // Iterate over every triangle defined by the index buffer
    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int idx0 = indices[i];
        unsigned int idx1 = indices[i + 1];
        unsigned int idx2 = indices[i + 2];

        glm::vec3 v0 = vertices[idx0].position;
        glm::vec3 v1 = vertices[idx1].position;
        glm::vec3 v2 = vertices[idx2].position;

        // Calculate Face Normal using Cross Product
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;

        // Avoid degenerate triangles (zero area) contributing NaNs
        glm::vec3 crossProd = glm::cross(edge1, edge2);
        if (glm::length(crossProd) < 1e-6f) continue;

        // Note: Not normalizing here allows larger triangles to contribute more weight,
        // which is often desirable. Or you can normalize to weight solely by angle.
        // Let's normalize to be safe against extreme aspect ratios.
        glm::vec3 faceNormal = glm::normalize(crossProd);

        vertices[idx0].normal += faceNormal;
        vertices[idx1].normal += faceNormal;
        vertices[idx2].normal += faceNormal;
    }

    // --- 3. Spatial Welding Pass ---
    // This is the key fix. We identify vertices that are at the same position 
    // and average their accumulated normals.

    // Key type for spatial hashing (Int coordinates)
    // Multiplier 10000.0 means precision up to 0.0001
    using PosKey = std::tuple<long long, long long, long long>;
    std::map<PosKey, glm::vec3> spatialNormalSum;
    const float PRECISION = 10000.0f;

    // Pass 3a: Sum normals for all spatially identical vertices
    for (const auto& v : vertices) {
        PosKey key = {
            static_cast<long long>(std::round(v.position.x * PRECISION)),
            static_cast<long long>(std::round(v.position.y * PRECISION)),
            static_cast<long long>(std::round(v.position.z * PRECISION))
        };
        spatialNormalSum[key] += v.normal;
    }

    // Pass 3b: Assign the averaged normal back to all vertices
    for (auto& v : vertices) {
        PosKey key = {
            static_cast<long long>(std::round(v.position.x * PRECISION)),
            static_cast<long long>(std::round(v.position.y * PRECISION)),
            static_cast<long long>(std::round(v.position.z * PRECISION))
        };

        // Get the total sum for this position
        glm::vec3 totalNormal = spatialNormalSum[key];

        // Normalize the final result
        if (glm::length(totalNormal) > 1e-6f) {
            v.normal = glm::normalize(totalNormal);
        }
        else {
            v.normal = glm::vec3(0.0f, 1.0f, 0.0f); // Fallback up vector
        }
    }
}

void GeometryRenderer::render(const glm::mat4& projection, const glm::mat4& view, const glm::mat4& model, const glm::vec3& cameraPos) {
    if (!shaderProgram) return;

    // [Fix] Force Enable Depth Test
    // ImGui or other libraries might disable it during their render pass.
    // We must ensure it's enabled before rendering our 3D geometry.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    // Optional: glDepthMask(GL_TRUE) if you have transparency logic elsewhere

    shaderProgram->use();
    shaderProgram->setMat4("projection", projection);
    shaderProgram->setMat4("view", view);
    shaderProgram->setMat4("model", model);

    shaderProgram->setVec3("lightPos", cameraPos);
    shaderProgram->setVec3("viewPos", cameraPos);
    shaderProgram->setVec3("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));

    for (size_t i = 0; i < geometries.size(); ++i) {
        if (i >= vaoHandles.size()) break;
        glBindVertexArray(vaoHandles[i]);

        // Handle AXES and GRID with GL_LINES (disjoint segments)
        if (geometries[i].type == GeometryType::AXES || geometries[i].type == GeometryType::GRID) {
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometries[i].vertices.size()));
        }
        // Handle CURVE with GL_LINE_STRIP (connected segments)
        else if (geometries[i].type == GeometryType::CURVE) {
            if (geometries[i].indices.empty()) {
                glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(geometries[i].vertices.size()));
            }
            else {
                glDrawElements(GL_LINE_STRIP, static_cast<GLsizei>(geometries[i].indices.size()), GL_UNSIGNED_INT, 0);
            }
        }
        // Handle SURFACE with GL_TRIANGLES
        else {
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(geometries[i].indices.size()), GL_UNSIGNED_INT, 0);
        }
    }
    glBindVertexArray(0);
}
