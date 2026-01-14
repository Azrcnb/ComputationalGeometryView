// Specify the GLSL version.
#version 330 core 

// Define input attributes for each vertex. These map to the data provided by glVertexAttribPointer calls on the CPU side.
layout (location = 0) in vec3 aPos;    // Attribute 0: Vertex position (x, y, z).
layout (location = 1) in vec3 aColor;  // Attribute 1: Vertex color (r, g, b).
layout (location = 2) in vec3 aNormal; // Attribute 2: Vertex normal vector (x, y, z).

// Define output variables that will be passed to the fragment shader. Interpolation occurs automatically across the primitive.
out vec3 FragPos;      // Output the world-space position of this vertex.
out vec3 Normal;       // Output the transformed normal vector for this vertex.
out vec3 vertexColor;  // Output the vertex color for this vertex.

// Uniform matrices used for transforming the vertex position and normal vector.
uniform mat4 model;      // Model matrix: Transforms from object/model space to world space.
uniform mat4 view;       // View matrix: Transforms from world space to view/camera space.
uniform mat4 projection; // Projection matrix: Transforms from view space to clip space.

void main()
{
    // Transform the incoming vertex position from object space to world space using the model matrix.
    // Store the result in FragPos for passing to the fragment shader.
    FragPos = vec3(model * vec4(aPos, 1.0)); 

    // Transform the incoming normal vector.
    // Simply multiplying by the model matrix can distort normals if there is non-uniform scaling.
    // The "Normal Matrix" (transpose of the inverse of the model matrix) correctly handles this transformation.
    Normal = mat3(transpose(inverse(model))) * aNormal;  

    // Pass the vertex color directly to the fragment shader.
    vertexColor = aColor; 
    
    // Calculate the final position of the vertex in clip space.
    // This chain of transformations is: Object -> World -> View -> Clip.
    // This value is automatically used by OpenGL for rasterization.
    gl_Position = projection * view * vec4(FragPos, 1.0); 
}