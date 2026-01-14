#pragma once // Include guard to prevent multiple inclusions of this header file

#include <glad/glad.h>   // Include GLAD for loading OpenGL function pointers
#include <GLFW/glfw3.h>  // Include GLFW for window management, input, etc.
#include <glm/glm.hpp>   // Include GLM core features (vectors, matrices)
#include <iostream>      // Include iostream for basic console output (cerr)
#include <functional>

#include "renderer/camera.h"          // Include header for the Camera class
#include "renderer/GeometryRenderer.h" // Include header for the GeometryRenderer class

/**
 * @brief Main application class responsible for setting up the OpenGL context,
 *        managing the main render loop, handling user input, and orchestrating
 *        the rendering process.
 */
class Application {
public:
    /**
     * @brief Constructor initializes member variables and sets up core systems like GLFW and OpenGL.
     */
    Application();

    /**
     * @brief Destructor cleans up resources allocated by GLFW.
     */
    ~Application();

    /**
     * @brief Entry point for running the application.
     *        Sets up callbacks and starts the main render loop.
     * @return An integer status code (0 for success).
     */
    int run();

    void setUICallback(std::function<void()> callback) {
        uiCallback = callback;
    }

    GeometryRenderer geometryRenderer; // Public instance of the renderer for drawing geometry

    GLFWwindow* getWindow() const { return window; }

private:
    // Private helper functions for initialization and the main loop
    void initGLFW();         // Initializes the GLFW library
    void initOpenGL();       // Creates a window, OpenGL context, and initializes GLAD
    void setupCallbacks();   // Registers GLFW callback functions for window events
    void renderLoop();       // The core loop where rendering and event polling occur
    void processInput();     // Handles keyboard input for application controls

    GLFWwindow* window; // Pointer to the GLFW window object

    Camera camera; // Instance of the camera class for 3D viewing

    // Timing variables for consistent animation/frame rate independent updates
    float deltaTime; // Time elapsed since the last frame (in seconds)
    float lastFrame; // Timestamp of the previous frame (in seconds)

    // Mouse tracking variables for camera control
    float lastX;      // Last recorded mouse X position
    float lastY;      // Last recorded mouse Y position
    bool firstMouse;  // Flag to handle the initial mouse movement event correctly

    // Static callback function declarations.
    // These are C-style functions required by GLFW. They receive the 'this' pointer
    // via glfwSetWindowUserPointer and cast it back to an Application instance.
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

    std::function<void()> uiCallback;
};