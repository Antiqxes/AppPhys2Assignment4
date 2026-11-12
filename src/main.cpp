#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <box2d/box2d.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {
    constexpr int kWindowWidth = 800;
    constexpr int kWindowHeight = 600;

    constexpr float kPixelsPerMeter = 32.0f;
    constexpr float kGravity = 9.81f;
    constexpr float kPi = 3.14159265f;

    constexpr float kStepDt = 1.0f / 60.0f;
    constexpr float kMaxFrameTime = 0.05f;
    constexpr int kVelocityIterations = 8;
    constexpr int kPositionIterations = 3;

    constexpr float kGroundX = 12.5f;
    constexpr float kGroundY = 1.0f;
    constexpr float kGroundHalfWidth = 12.5f;
    constexpr float kGroundHalfHeight = 0.5f;
    constexpr float kGroundTop = kGroundY + kGroundHalfHeight;

    constexpr float kPivotX = 3.0f;
    constexpr float kPivotY = 2.5f;
    constexpr float kBaseHalfWidth = 0.8f;
    constexpr float kBaseHalfHeight = 0.5f;

    constexpr float kBarrelLength = 1.5f;
    constexpr float kBarrelHalfThickness = 0.25f;
    constexpr float kMinAngle = 5.0f * kPi / 180.0f;
    constexpr float kMaxAngle = 85.0f * kPi / 180.0f;
    constexpr float kMuzzleSize = 8.0f;

    constexpr float kMinPower = 20.0f;
    constexpr float kMaxPower = 40.0f;
    constexpr float kChargeRate = 20.0f;
    constexpr float kBarX = 20.0f;
    constexpr float kBarY = 580.0f;
    constexpr float kBarWidth = 200.0f;
    constexpr float kBarHeight = 16.0f;

    constexpr float kBlockHalf = 0.4f;
    constexpr float kTowerLeftX = 19.0f;
    constexpr int kTowerColumns = 3;
    constexpr int kTowerRows = 5;

    constexpr float kKnockDistance = 0.3f;

    constexpr float kLineThickness = 2.0f;
    constexpr int kCircleSegments = 24;

    constexpr float kBallRadius = 0.2f;
    constexpr float kBallDensity = 3.0f;

    constexpr float kWorldWidth = kWindowWidth / kPixelsPerMeter;
    constexpr float kRemoveBelowY = -10.0f;

    struct Block {
        b2Body* body;
        b2Vec2 start;
        bool knocked;
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

    b2Vec2 muzzlePoint(float angle) {
        return b2Vec2(kPivotX + kBarrelLength * std::cos(angle),
            kPivotY + kBarrelLength * std::sin(angle));
    }

    void drawBarrel(float angle) {
        gfx::drawBox(metersToPixelsX(kPivotX + 0.5f * kBarrelLength * std::cos(angle)),
            metersToPixelsY(kPivotY + 0.5f * kBarrelLength * std::sin(angle)),
            kBarrelLength * kPixelsPerMeter, kBarrelHalfThickness * 2.0f * kPixelsPerMeter,
            -angle, 0.6f, 0.6f, 0.7f);
    }

    void drawMuzzle(float angle) {
        b2Vec2 muzzle = muzzlePoint(angle);
        gfx::drawBox(metersToPixelsX(muzzle.x), metersToPixelsY(muzzle.y), kMuzzleSize, kMuzzleSize,
            0.0f, 1.0f, 0.9f, 0.3f);
    }

    void drawPowerBar(float power) {
        float fraction = (power - kMinPower) / (kMaxPower - kMinPower);
        gfx::drawBox(kBarX + kBarWidth * 0.5f, kBarY, kBarWidth, kBarHeight, 0.0f, 0.25f, 0.25f,
            0.3f);
        gfx::drawBox(kBarX + kBarWidth * fraction * 0.5f, kBarY, kBarWidth * fraction, kBarHeight,
            0.0f, 0.31f, 0.76f, 0.97f);
    }

    void buildTower(b2World& world, std::vector<Block>& blocks) {
        for (int column = 0; column < kTowerColumns; ++column) {
            for (int row = 0; row < kTowerRows; ++row) {
                b2BodyDef bodyDef;
                bodyDef.type = b2_dynamicBody;
                bodyDef.position.Set(kTowerLeftX + (column + 0.5f) * 2.0f * kBlockHalf,
                    kGroundTop + (row + 0.5f) * 2.0f * kBlockHalf);
                b2Body* body = world.CreateBody(&bodyDef);

                b2PolygonShape shape;
                shape.SetAsBox(kBlockHalf, kBlockHalf);

                b2FixtureDef fixtureDef;
                fixtureDef.shape = &shape;
                fixtureDef.density = 1.0f;
                fixtureDef.friction = 0.6f;
                fixtureDef.restitution = 0.0f;
                body->CreateFixture(&fixtureDef);

                blocks.push_back({ body, body->GetPosition(), false });
            }
        }
    }

    // A line is a thin box laid between two points.
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

    void drawCircle(b2Vec2 center, float radius, float r, float g, float b) {
        for (int i = 0; i < kCircleSegments; ++i) {
            float a1 = 2.0f * kPi * i / kCircleSegments;
            float a2 = 2.0f * kPi * (i + 1) / kCircleSegments;
            drawLine(b2Vec2(center.x + radius * std::cos(a1), center.y + radius * std::sin(a1)),
                b2Vec2(center.x + radius * std::cos(a2), center.y + radius * std::sin(a2)), r, g,
                b);
        }
    }

    b2Body* createBall(b2World& world, b2Vec2 position) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = position;
        b2Body* body = world.CreateBody(&bodyDef);

        b2CircleShape shape;
        shape.m_radius = kBallRadius;

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.density = kBallDensity;
        fixtureDef.friction = 0.4f;
        fixtureDef.restitution = 0.2f;
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

        std::vector<Block> blocks;
        buildTower(world, blocks);

        std::vector<b2Body*> balls;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        float power = kMinPower;
        bool charging = false;
        std::string lastTitle;

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            double now = glfwGetTime();
            float frameTime = std::min(static_cast<float>(now - previousTime), kMaxFrameTime);
            previousTime = now;

            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            float dx = pixelsToMetersX(static_cast<float>(mouseX)) - kPivotX;
            float dy = pixelsToMetersY(static_cast<float>(mouseY)) - kPivotY;
            float aimAngle = std::clamp(std::atan2(dy, dx), kMinAngle, kMaxAngle);

            bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            if (mouseDown) {
                charging = true;
                power = std::min(power + kChargeRate * frameTime, kMaxPower);
            }
            else if (charging) {
                b2Body* ball = createBall(world, muzzlePoint(aimAngle));
                float impulse = ball->GetMass() * power;
                ball->ApplyLinearImpulseToCenter(
                    b2Vec2(impulse * std::cos(aimAngle), impulse * std::sin(aimAngle)), true);
                balls.push_back(ball);
                charging = false;
                power = kMinPower;
            }

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, kVelocityIterations, kPositionIterations);
                accumulator -= kStepDt;
            }

            int knockedCount = 0;
            for (Block& block : blocks) {
                if ((block.body->GetPosition() - block.start).Length() > kKnockDistance) {
                    block.knocked = true;
                }
                if (block.knocked) {
                    ++knockedCount;
                }
            }

            for (auto it = balls.begin(); it != balls.end();) {
                b2Vec2 position = (*it)->GetPosition();
                if (position.x < 0.0f || position.x > kWorldWidth || position.y < kRemoveBelowY) {
                    world.DestroyBody(*it);
                    it = balls.erase(it);
                }
                else {
                    ++it;
                }
            }

            std::string title = std::string("Cannon | knocked: ") + std::to_string(knockedCount) +
                "/" + std::to_string(blocks.size());
            if (title != lastTitle) {
                glfwSetWindowTitle(glfwGetCurrentContext(), title.c_str());
                lastTitle = title;
            }

            glClear(GL_COLOR_BUFFER_BIT);

            drawBoxBody(ground, kGroundHalfWidth, kGroundHalfHeight, 0.35f, 0.35f, 0.4f);
            drawBoxBody(base, kBaseHalfWidth, kBaseHalfHeight, 0.35f, 0.35f, 0.4f);
            drawBarrel(aimAngle);
            drawMuzzle(aimAngle);
            drawPowerBar(power);
            for (const Block& block : blocks) {
                float r = 1.0f, g = 0.6f, b = 0.2f;
                if (block.knocked) {
                    r = 0.2f, g = 1.0f, b = 0.6f;
                }
                if (!block.body->IsAwake()) {
                    r *= 0.5f;
                    g *= 0.5f;
                    b *= 0.5;
                }
                drawBoxBody(block.body, kBlockHalf, kBlockHalf, r, g, b);
            }
            for (const b2Body* ball : balls) {
                drawCircle(ball->GetPosition(), kBallRadius, 1.0f, 0.4f, 0.4f);
            }

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