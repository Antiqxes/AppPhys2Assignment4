#include "core/input.h"

#include <GLFW/glfw3.h>

#include "core/window.h"

namespace gfx {

    InputState processInput(Window& window) {
        if (window.isKeyPressed(GLFW_KEY_ESCAPE)) {
            window.close();
        }

        InputState input;

        // 1. WASD movement
        if (window.isKeyPressed(GLFW_KEY_W)) input.move.y += 1.0f;
        if (window.isKeyPressed(GLFW_KEY_S)) input.move.y -= 1.0f;
        if (window.isKeyPressed(GLFW_KEY_A)) input.move.x -= 1.0f;
        if (window.isKeyPressed(GLFW_KEY_D)) input.move.x += 1.0f;

        if (input.move.LengthSquared() > 0.0f) {
            input.move.Normalize();
        }

        // 2. Mouse cursor position in pixels
        GLFWwindow* rawWindow = glfwGetCurrentContext();
        double mouseX = 0.0;
        double mouseY = 0.0;
        glfwGetCursorPos(rawWindow, &mouseX, &mouseY);
        input.mousePixelPos.Set(static_cast<float>(mouseX), static_cast<float>(mouseY));

        // 3. Pull / Hold (Left Mouse Button or Spacebar)
        input.pullActive = window.isKeyPressed(GLFW_KEY_SPACE) ||
            (glfwGetMouseButton(rawWindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);

        // 4. Punt / Repel (Right Mouse Button or E key - fires once per press)
        static bool wasRepelDown = false;
        bool repelDown = window.isKeyPressed(GLFW_KEY_E) ||
            (glfwGetMouseButton(rawWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        input.repelTriggered = repelDown && !wasRepelDown;
        wasRepelDown = repelDown;

        return input;
    }

}  // namespace gfx
