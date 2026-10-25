#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <box2d/box2d.h>

#include<algorithm>
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

constexpr float kBoxHalfSize = 1.0f;

constexpr float kStepDt = 1.0f / 60.0f;
constexpr float kMaxFrameTime = 0.05f;

constexpr int kVelocityIterations = 8;
constexpr int kPositionIterations = 3;

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

        b2BodyDef boxDef;
        boxDef.type = b2_dynamicBody;
        boxDef.position.Set(12.5f, 14.0f);
        b2Body* box = world.CreateBody(&boxDef);

        b2PolygonShape boxShape;
        boxShape.SetAsBox(kBoxHalfSize, kBoxHalfSize);

        b2FixtureDef boxFixture;
        boxFixture.shape = &boxShape;
        boxFixture.density = 1.0f;
        boxFixture.friction = 0.3f;
        box->CreateFixture(&boxFixture);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        bool wasMouseDown = false;

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            double now = glfwGetTime();
            float frameTime = std::min(static_cast<float>(now - previousTime), kMaxFrameTime);
            previousTime = now;



            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            if (mouseDown && !wasMouseDown) {
                std::cout << "click at " << mouseX << ", " << mouseY << '\n';
            }
            wasMouseDown = mouseDown;

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, kVelocityIterations, kPositionIterations);
                accumulator -= kStepDt;
            }


            glClear(GL_COLOR_BUFFER_BIT);

            b2Vec2 groundPosition = ground->GetPosition();

            gfx::drawBox(groundPosition.x * 32.0f, kWindowHeight - groundPosition.y * 32.0f,
                kGroundHalfWidth * 2.0f * 32.0f, kGroundHalfHeight * 2.0f * 32.0f, 0.0f,
                0.35f, 0.35f, 0.4f);

            b2Vec2 boxPosition = box->GetPosition();
            gfx::drawBox(boxPosition.x * 32.0f, kWindowHeight - boxPosition.y * 32.0f,
                kBoxHalfSize * 2.0f * 32.0f, kBoxHalfSize * 2.0f * 32.0f, 0.0f, 1.0f,
                0.5f, 0.2f);


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
