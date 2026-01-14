// Specify the GLSL version. Version 330 is part of OpenGL 3.3.
#version 330 core 

// Define the output variable for the final fragment color that will be written to the framebuffer.
out vec4 FragColor;

// Input variables received from the vertex shader's outputs after interpolation across the primitive.
in vec3 FragPos;      // The interpolated world-space position of the current fragment.
in vec3 Normal;       // The interpolated normal vector at the current fragment.
in vec3 vertexColor;  // The interpolated color value from the vertex.

// Uniforms are global variables passed from the application (CPU side) to the shader. They remain constant for all fragments in a draw call.
uniform vec3 lightPos;   // The world-space position of the light source.
uniform vec3 viewPos;    // The world-space position of the camera/viewer.
uniform vec3 lightColor; // The color/intensity of the light source.

void main()
{
    // 0. Special handling: If this is a wireframe or an axis line (normal length is close to 0),
    // directly output the vertex color without lighting calculations.
    // This check helps avoid artifacts when rendering non-surface geometry like debug lines.
    if (length(Normal) < 0.1) { // If the length of the normal vector is very small (e.g., for lines/points)
         FragColor = vec4(vertexColor, 1.0); // Output the vertex color with full alpha.
         return; // Exit the function early, skipping lighting.
    }

    // Calculate the direction vector from the fragment position towards the light source.
    vec3 lightDir = normalize(lightPos - FragPos); 
    // Normalize the input normal vector to ensure it has unit length for accurate lighting calculations.
    vec3 norm = normalize(Normal);

    // ==========================================================
    // CRITICAL FIX: Two-Sided Lighting Logic
    // ==========================================================
    // Determine if the surface we are shading is facing away from the light.
    // This happens when the angle between the normal and light direction is greater than 90 degrees (dot product is negative).
    // In a headlamp-style lighting setup (like a flashlight attached to the camera),
    // the light effectively shines on the backside of faces pointed away from the viewer.
    // To simulate this, we flip the normal vector so it points towards the light (and thus also effectively towards the viewer in this context).
    if (dot(norm, lightDir) < 0.0) { // Check if the angle is obtuse (> 90 degrees)
        norm = -norm; // Flip the normal vector to point towards the light.
    }
    // ==========================================================

    // 1. Ambient Light Calculation:
    // Ambient light simulates indirect lighting that illuminates all surfaces evenly, preventing them from being completely black in shadow.
    float ambientStrength = 0.3; // A small factor for ambient light intensity.
    vec3 ambient = ambientStrength * lightColor; // Calculate the ambient contribution based on light color.

    // 2. Diffuse Light Calculation (Lambertian Reflection):
    // Diffuse reflection determines how much light is scattered equally in all directions from a rough surface.
    // It depends on the angle between the surface normal and the light direction.
    // After the normal flip above, 'norm' now points towards the light.
    float diff = max(dot(norm, lightDir), 0.0); // Calculate the cosine of the angle. Clamp to 0 to ignore back-faces.
    vec3 diffuse = diff * lightColor; // Calculate the diffuse contribution, scaled by light color.

    // 3. Specular Light Calculation (Phong Reflection Model):
    // Specular reflection simulates bright highlights on shiny surfaces.
    // It depends on the angle between the reflected light ray and the viewer's direction.
    float specularStrength = 0.5; // Factor controlling the intensity of the specular highlight.
    vec3 viewDir = normalize(viewPos - FragPos); // Calculate the direction from the fragment towards the camera.
    vec3 reflectDir = reflect(-lightDir, norm);  // Calculate the perfect reflection vector of the light off the surface (using the potentially flipped normal).
    // Calculate the specular factor using the Phong model.
    // The dot product gives the cosine of the angle between the view direction and the reflected light direction.
    // Raising it to a power ('shininess') creates a sharp, concentrated highlight.
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32); 
    vec3 specular = specularStrength * spec * lightColor; // Calculate the final specular contribution.

    // Combine the three components of the lighting model (ambient, diffuse, specular).
    vec3 result = (ambient + diffuse + specular) * vertexColor; // Multiply the total light by the fragment's original color.
    // Set the final output color for this fragment.
    FragColor = vec4(result, 1.0); // Output the calculated color with alpha set to 1.0 (fully opaque).
}