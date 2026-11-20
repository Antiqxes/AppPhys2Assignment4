#pragma once

#include <box2d/b2_math.h>

namespace gfx {

    class Window;

    struct InputState {
        b2Vec2 move{ 0.0f, 0.0f };
        bool magnetActive = false;
    };

    InputState processInput(Window& window);

}  // namespace gfx

