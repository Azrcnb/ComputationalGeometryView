#include "Application.h" 
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/type_ptr.hpp>         
#include <iostream>

// Include ImGui headers
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// [Fix] Forward declaration of ImGui's internal GLFW callback handlers.
// These are needed because we override the GLFW callbacks in 'setupCallbacks',
// so we must manually forward the events to ImGui.
extern IMGUI_IMPL_API void ImGui_ImplGlfw_CursorPosCallback(GLFWwindow* window, double x, double y);
extern IMGUI_IMPL_API void ImGui_ImplGlfw_MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
extern IMGUI_IMPL_API void ImGui_ImplGlfw_ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

// Constructor: Initializes member variables and calls setup functions
Application::Application()
    : window(nullptr), deltaTime(0.0f), lastFrame(0.0f),
    camera(glm::vec3(0.0f, 5.0f, 10.0f), 20.0f), // Initialize camera at (0, 5, 10)
    lastX(400.0f), lastY(300.0f), firstMouse(true)
{
    initGLFW();
    initOpenGL();
}

// Destructor: Cleans up GLFW and ImGui resources
Application::~Application() {
    // Explicitly shutdown ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

// Initializes the GLFW library and sets window hints
void Application::initGLFW() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        exit(EXIT_FAILURE);
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
}

// Creates the GLFW window, loads GLAD, initializes ImGui and the renderer
void Application::initOpenGL() {
    // Create window with slightly larger dimensions for better UI visibility
    window = glfwCreateWindow(1280, 720, "Computational Geometry View", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, this);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        exit(EXIT_FAILURE);
    }

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    geometryRenderer.init();

    // Initialize ImGui Context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    // Initialize ImGui backends
    // Note: This installs ImGui's callbacks, but we will overwrite them in 'run()',
    // hence the need for manual forwarding later.
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

// Main function to run the application
int Application::run() {
    setupCallbacks(); // This overwrites ImGui's callbacks
    renderLoop();
    return 0;
}

// Registers the static callback functions with GLFW
void Application::setupCallbacks() {
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
}

// -----------------------------------------------------------
// Static Callback Implementations
// -----------------------------------------------------------

void Application::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void Application::mouse_callback(GLFWwindow* window, double xposIn, double yposIn) {
    // [Fix] Forward event to ImGui first
    ImGui_ImplGlfw_CursorPosCallback(window, xposIn, yposIn);

    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app) return;

    // [Logic] If ImGui wants to capture the mouse (e.g., hovering over a window),
    // do not process camera rotation.
    if (ImGui::GetIO().WantCaptureMouse) return;

    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (app->firstMouse) {
        app->lastX = xpos;
        app->lastY = ypos;
        app->firstMouse = false;
    }

    float xoffset = xpos - app->lastX;
    float yoffset = app->lastY - ypos;

    app->lastX = xpos;
    app->lastY = ypos;

    app->camera.ProcessMouseMovement(xoffset, yoffset);
}

void Application::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    // [Fix] Forward event to ImGui first
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);

    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    // [Logic] Only zoom camera if ImGui is not capturing the mouse
    if (app && !ImGui::GetIO().WantCaptureMouse) {
        app->camera.ProcessMouseScroll(static_cast<float>(yoffset));
    }
}

void Application::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    // [Fix] Essential: Forward click events to ImGui.
    // Without this, clicking on ImGui widgets (like the Combo box) will fail.
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    // [Logic] Process application clicks only if ImGui is not focused
    if (app && !ImGui::GetIO().WantCaptureMouse) {
        app->camera.ProcessMouseButton(button, action);
    }
}

// The main rendering loop
void Application::renderLoop() {
    while (!glfwWindowShouldClose(window)) {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput();

        // --- ImGui Frame Start ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Execute user-defined UI logic (injected from main.cpp)
        if (uiCallback) {
            uiCallback();
        }

        // --- ImGui Internal Rendering ---
        ImGui::Render();

        // --- 3D Scene Rendering ---
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        if (height == 0) height = 1;

        // Setup matrices
        glm::mat4 projection = glm::perspective(
            glm::radians(45.0f),
            (float)width / (float)height,
            0.1f,
            100.0f
        );

        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 model = glm::mat4(1.0f);

        // Render Geometry
        geometryRenderer.render(projection, view, model, camera.GetPosition());

        // --- Draw ImGui Data Overlay ---
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

// Handles keyboard input
void Application::processInput() {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
    // Camera movement logic can be added here if needed

    // Camera Movement Controls
    // W/S: Move Forward/Backward
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);

    // A/D: Move Left/Right
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);

    // Q/E: Move Down/Up (Standard in 3D editors)
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);
}
