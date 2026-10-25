#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <box2d/box2d.h>

#include <cstdlib>
#include <exception>
#include <iostream>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;

constexpr float kGravity = -9.8f;

constexpr float kGroundX = 12.5f;
constexpr float kGroundY = 1.0f;
constexpr float kGroundHalfWidth = 10.0f;
constexpr float kGroundHalfHeight = 0.5f;

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight, .title = "Box2D"});
        gfx::initDraw(kWindowWidth, kWindowHeight);

		b2World world(b2Vec2(0.0f, kGravity));

        b2BodyDef groundDef;

        groundDef.position.Set(kGroundX, kGroundY);
        b2Body* ground = world.CreateBody(&groundDef);

        b2PolygonShape groundShape;

        groundShape.SetAsBox(kGroundHalfWidth, kGroundHalfHeight);

        b2FixtureDef groundFixture;
        groundFixture.shape = &groundShape;
        groundFixture.friction = 0.6f;
        ground->CreateFixture(&groundFixture);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        bool wasMouseDown = false;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            if (mouseDown && !wasMouseDown) {
                std::cout << "click at " << mouseX << ", " << mouseY << '\n';
            }
            wasMouseDown = mouseDown;

            glClear(GL_COLOR_BUFFER_BIT);

            b2Vec2 groundPosition = ground->GetPosition();

            gfx::drawBox(groundPosition.x * 32.0f, kWindowHeight - groundPosition.y * 32.0f,
                kGroundHalfWidth * 2.0f * 32.0f, kGroundHalfHeight * 2.0f * 32.0f, 0.0f,
                0.35f, 0.35f, 0.4f);

            window.swapBuffers();
            window.pollEvents();
        }

        gfx::shutdownDraw();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
