#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <box2d/box2d.h>

#include <algorithm>

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

    constexpr float kPixelsPerMeter = 32.0f;
    constexpr float kGravity = 9.81f;


    constexpr float kStepDt = 1.0f / 60.0f;
    constexpr float kMaxFrameTime = 0.05f;
    constexpr int kVelocityIterations = 8;
    constexpr int kPositionIterations = 3;


    constexpr float kGroundX = 12.5f;
    constexpr float kGroundY = 1.0f;
    constexpr float kGroundHalfWidth = 12.5f;
    constexpr float kGroundHalfHeight = 0.5f;

    constexpr float kPivotX = 3.0f;
    constexpr float kPivotY = 2.5f;
    constexpr float kBaseHalfWidth = 0.8f;
    constexpr float kBaseHalfHeight = 0.5f;

    float metersToPixelsX(float xMeters) {
        return xMeters * kPixelsPerMeter;
    }

    float metersToPixelsY(float yMeters) {
        return kWindowHeight - yMeters * kPixelsPerMeter;
    }

    void drawBoxBody(const b2Body* body, float halfWidth, float halfHeight, float r, float g,
        float b) {
        b2Vec2 position = body->GetPosition();
        gfx::drawBox(metersToPixelsX(position.x), metersToPixelsY(position.y),
            halfWidth * 2.0f * kPixelsPerMeter, halfHeight * 2.0f * kPixelsPerMeter,
            -body->GetAngle(), r, g, b);
    }

    b2Body* createStaticBox(b2World& world, float x, float y, float halfWidth, float halfHeight) {
        b2BodyDef bodyDef;
        bodyDef.position.Set(x, y);
        b2Body* body = world.CreateBody(&bodyDef);

        b2PolygonShape shape;
        shape.SetAsBox(halfWidth, halfHeight);

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.friction = 0.6f;
        body->CreateFixture(&fixtureDef);

        return body;
    }

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({ .width = kWindowWidth, .height = kWindowHeight, .title = "Box2D" });
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world(b2Vec2(0.0f, -kGravity));

        b2Body* ground =
            createStaticBox(world, kGroundX, kGroundY, kGroundHalfWidth, kGroundHalfHeight);
        b2Body* base = createStaticBox(world, kPivotX, kPivotY - kBaseHalfHeight, kBaseHalfWidth,
            kBaseHalfHeight);


        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            double now = glfwGetTime();
            float frameTime = std::min(static_cast<float>(now - previousTime), kMaxFrameTime);
            previousTime = now;

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, kVelocityIterations, kPositionIterations);
                accumulator -= kStepDt;
            }

            glClear(GL_COLOR_BUFFER_BIT);

            drawBoxBody(ground, kGroundHalfWidth, kGroundHalfHeight, 0.35f, 0.35f, 0.4f);
            drawBoxBody(base, kBaseHalfWidth, kBaseHalfHeight, 0.35f, 0.35f, 0.4f);

            window.swapBuffers();
            window.pollEvents();
        }

        gfx::shutdownDraw();
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}