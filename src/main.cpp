#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <box2d/box2d.h>

#include<algorithm>
#include<cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include<vector>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;

constexpr float kPixelsPerMeter = 32.0f;

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

constexpr float kSpawnCooldown = 0.25f;
constexpr float kSpawnHalfSize = 0.5f;

constexpr float kRemoveBelowY = -10.0f;

constexpr float kLineThickness = 2.0f;
constexpr float kAxisLength = 0.6f;

struct Box {
    b2Body* body;
    float halfWidth;
    float halfHeight;
    float r, g, b;
};

float metersToPixelsX(float xMeters) {
    return xMeters * kPixelsPerMeter;
}

float metersToPixelsY(float yMeters) {
    return kWindowHeight - yMeters * kPixelsPerMeter;
}

float pixelsToMetersX(float xPixels) {
    return xPixels / kPixelsPerMeter;
}

float pixelsToMetersY(float yPixels) {
    return (kWindowHeight - yPixels) / kPixelsPerMeter;
}

void drawBoxBody(const b2Body* body, float halfWidth, float halfHeight, float r, float g,
    float b) {
    b2Vec2 position = body->GetPosition();
    gfx::drawBox(metersToPixelsX(position.x), metersToPixelsY(position.y),
        halfWidth * 2.0f * kPixelsPerMeter, halfHeight * 2.0f * kPixelsPerMeter,
        -body->GetAngle(), r, g, b);
}

void drawLine(b2Vec2 start, b2Vec2 end, float r, float g, float b) {
	float x1 = metersToPixelsX(start.x);
	float y1 = metersToPixelsY(start.y);
	float x2 = metersToPixelsX(end.x);
	float y2 = metersToPixelsY(end.y);
	float dx = x2 - x1;
	float dy = y2 - y1;
    gfx::drawBox((x1 + x2) * 0.5f, (y1 + y2) * 0.5f, std::sqrt(dx * dx + dy * dy) + kLineThickness,
    kLineThickness, std::atan2(dy, dx), r, g, b);
}

void drawAxes(const b2Body* body) {
    const b2Transform& transform = body->GetTransform();
    drawLine(transform.p, b2Mul(transform, b2Vec2(kAxisLength, 0.0f)), 1.0f, 0.3f, 0.3f);
    drawLine(transform.p, b2Mul(transform, b2Vec2(0.0f, kAxisLength)), 0.3f, 1.0f, 0.3f);
}

Box createBox(b2World& world, float x, float y, float halfWidth, float halfHeight,float density, float friction,
    float restitution, float r, float g, float b) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(x, y);
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(halfWidth, halfHeight);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = density;
    fixtureDef.friction = friction;
    fixtureDef.restitution = restitution;
    body->CreateFixture(&fixtureDef);

    return { body, halfWidth, halfHeight, r, g, b };
}

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

        std::vector<Box> boxes;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        float spawnTimer = 0.0f;

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
            spawnTimer -= frameTime;
            if (mouseDown && spawnTimer <= 0.0f) {
                boxes.push_back(createBox(world, pixelsToMetersX(static_cast<float>(mouseX)),
                    pixelsToMetersY(static_cast<float>(mouseY)),
                    kSpawnHalfSize, kSpawnHalfSize, 4.0f, 0.5f, 0.3f, 1.0f, 0.5f,
                    0.2f));
                spawnTimer = kSpawnCooldown;
            }

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, kVelocityIterations, kPositionIterations);
                accumulator -= kStepDt;
            }

            for (auto it = boxes.begin(); it != boxes.end();) {
                if (it->body->GetPosition().y < kRemoveBelowY) {
                    world.DestroyBody(it->body);
                    it = boxes.erase(it);
                }
                else {
                    ++it;
                }
            }

            glClear(GL_COLOR_BUFFER_BIT);

            drawBoxBody(ground, kGroundHalfWidth, kGroundHalfHeight, 0.35f, 0.35f, 0.4f);
            for (const Box& box : boxes) {
                drawBoxBody(box.body, box.halfWidth, box.halfHeight, box.r, box.g, box.b);
                drawAxes(box.body);
            }

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
