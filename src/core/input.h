#pragma once

#include <box2d/b2_math.h>

namespace gfx {

    class Window;

    struct InputState {
        b2Vec2 move{ 0.0f, 0.0f };
        b2Vec2 mousePixelPos{ 0.0f, 0.0f };
        bool pullActive = false;
        bool repelTriggered = false;
    };

    InputState processInput(Window& window);

}  // namespace gfx
