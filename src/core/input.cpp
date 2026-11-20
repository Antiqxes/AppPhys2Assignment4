#include "core/input.h"

#include <GLFW/glfw3.h>

#include "core/window.h"

namespace gfx {

    InputState processInput(Window& window) {
        if (window.isKeyPressed(GLFW_KEY_ESCAPE)) {
            window.close();
        }

        InputState input;
        if (window.isKeyPressed(GLFW_KEY_W)) input.move.y += 1.0f;
        if (window.isKeyPressed(GLFW_KEY_S)) input.move.y -= 1.0f;
        if (window.isKeyPressed(GLFW_KEY_A)) input.move.x -= 1.0f;
        if (window.isKeyPressed(GLFW_KEY_D)) input.move.x += 1.0f;

        if (input.move.LengthSquared() > 0.0f) {
            input.move.Normalize();
        }

        input.magnetActive = window.isKeyPressed(GLFW_KEY_SPACE);
        return input;
    }

}  // namespace gfx
