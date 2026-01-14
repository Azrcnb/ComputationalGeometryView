#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Defines several possible options for camera movement
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 10.0f; // [New] Movement speed
const float SENSITIVITY = 0.5f;
const float ZOOM_SENS = 1.0f;
const float MIN_DIST = 1.0f;
const float MAX_DIST = 100.0f;

class Camera
{
public:
    // Camera Attributes
    glm::vec3 Target;
    float Distance;

    // Euler Angles
    float Yaw;
    float Pitch;

    // Camera options
    float MovementSpeed;
    float MouseSensitivity;
    float ZoomSensitivity;

    // Interaction flags
    bool isRotating = false;
    bool isPanning = false;

    // Constructor with vectors
    Camera(glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f), float distance = 20.0f)
        : Target(target), Distance(distance), Yaw(YAW), Pitch(PITCH),
        MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), ZoomSensitivity(ZOOM_SENS)
    {
    }

    // Returns the view matrix calculated using Euler Angles and the LookAt Matrix
    glm::mat4 GetViewMatrix()
    {
        return glm::lookAt(GetPosition(), Target, glm::vec3(0.0f, 1.0f, 0.0f));
    }

    // Calculate world position from orbit parameters
    glm::vec3 GetPosition()
    {
        float radYaw = glm::radians(Yaw);
        float radPitch = glm::radians(Pitch);

        float x = Distance * cos(radPitch) * cos(radYaw);
        float y = Distance * sin(radPitch);
        float z = Distance * cos(radPitch) * sin(radYaw);

        return Target + glm::vec3(x, y, z);
    }

    // [New] Processes input received from any keyboard-like input system
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;

        // Calculate basis vectors based on current viewing angle (orbit)
        // Note: Front vector points FROM camera TO target
        glm::vec3 front = glm::normalize(Target - GetPosition());
        glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f); // World Up

        // Move the Target point, camera follows because it orbits the target
        if (direction == FORWARD)
            Target += front * velocity;
        if (direction == BACKWARD)
            Target -= front * velocity;
        if (direction == LEFT)
            Target -= right * velocity;
        if (direction == RIGHT)
            Target += right * velocity;
        if (direction == UP)
            Target += up * velocity;
        if (direction == DOWN)
            Target -= up * velocity;
    }

    // Processes input received from a mouse input system
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
    {
        if (isRotating)
        {
            xoffset *= MouseSensitivity;
            yoffset *= MouseSensitivity;

            Yaw += xoffset;
            Pitch -= yoffset;

            if (constrainPitch)
            {
                if (Pitch > 89.0f) Pitch = 89.0f;
                if (Pitch < -89.0f) Pitch = -89.0f;
            }
        }
        else if (isPanning)
        {
            float panSpeed = Distance * 0.001f;
            glm::vec3 front = glm::normalize(Target - GetPosition());
            glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0, 1, 0)));
            glm::vec3 up = glm::normalize(glm::cross(right, front));

            Target -= right * (xoffset * panSpeed);
            Target -= up * (yoffset * panSpeed);
        }
    }

    // Processes mouse scroll
    void ProcessMouseScroll(float yoffset)
    {
        Distance -= yoffset * ZoomSensitivity;
        if (Distance < MIN_DIST) Distance = MIN_DIST;
        if (Distance > MAX_DIST) Distance = MAX_DIST;
    }

    void ProcessMouseButton(int button, int action)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)   isRotating = (action == GLFW_PRESS);
        if (button == GLFW_MOUSE_BUTTON_MIDDLE) isPanning = (action == GLFW_PRESS);
    }
};
#endif
