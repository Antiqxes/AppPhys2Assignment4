#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <box2d/box2d.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
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

    constexpr float kGroundX = 12.5f;
    constexpr float kGroundY = 1.0f;
    constexpr float kGroundHalfWidth = 10.0f;
    constexpr float kGroundHalfHeight = 0.5f;

    constexpr float kStepDt = 1.0f / 60.0f;
    constexpr float kMaxFrameTime = 0.05f;
    constexpr int kVelocityIterations = 8;
    constexpr int kPositionIterations = 3;

    constexpr float kSpawnCooldown = 0.25f;
    constexpr float kRemoveBelowY = -10.0f;

    constexpr float kLineThickness = 2.0f;
    constexpr float kAxisLength = 0.6f;
    constexpr int kCircleSegments = 24;

    constexpr float kShapeSize = 0.5f;
    constexpr float kPolygonRadius = 0.6f;

    struct ShapeStyle {
        int sides;
        float r, g, b;
    };

    constexpr ShapeStyle kShapes[] = {
        {4, 1.0f, 0.6f, 0.2f},
        {0, 0.4f, 0.8f, 1.0f},
        {3, 1.0f, 0.9f, 0.3f},
        {5, 0.8f, 0.5f, 1.0f},
        {6, 0.5f, 1.0f, 0.6f},
    };
    constexpr int kShapeCount = 5;

    struct Item {
        b2Body* body;
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

    void drawAxes(const b2Body* body) {
        const b2Transform& transform = body->GetTransform();
        drawLine(transform.p, b2Mul(transform, b2Vec2(kAxisLength, 0.0f)), 1.0f, 0.3f, 0.3f);
        drawLine(transform.p, b2Mul(transform, b2Vec2(0.0f, kAxisLength)), 0.3f, 1.0f, 0.3f);
    }

    void drawItem(const Item& item) {
        const b2Transform& transform = item.body->GetTransform();
        for (const b2Fixture* fixture = item.body->GetFixtureList(); fixture;
            fixture = fixture->GetNext()) {
            const b2Shape* shape = fixture->GetShape();
            if (shape->GetType() == b2Shape::e_circle) {
                const b2CircleShape* circle = static_cast<const b2CircleShape*>(shape);
                drawCircle(b2Mul(transform, circle->m_p), circle->m_radius, item.r, item.g, item.b);
            }
            else if (shape->GetType() == b2Shape::e_polygon) {
                const b2PolygonShape* polygon = static_cast<const b2PolygonShape*>(shape);
                for (int i = 0; i < polygon->m_count; ++i) {
                    int next = (i + 1) % polygon->m_count;
                    drawLine(b2Mul(transform, polygon->m_vertices[i]),
                        b2Mul(transform, polygon->m_vertices[next]), item.r, item.g, item.b);
                }
            }
        }
        drawAxes(item.body);
    }

    Item createItem(b2World& world, const ShapeStyle& style, float x, float y) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(x, y);
        b2Body* body = world.CreateBody(&bodyDef);

        b2CircleShape circle;
        b2PolygonShape polygon;
        b2FixtureDef fixtureDef;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = 0.5f;
        fixtureDef.restitution = 0.3f;

        if (style.sides == 0) {
            circle.m_radius = kShapeSize;
            fixtureDef.shape = &circle;
        }
        else if (style.sides == 4) {
            polygon.SetAsBox(kShapeSize, kShapeSize);
            fixtureDef.shape = &polygon;
        }
        else {
            // Corners sit on a circle, counter-clockwise, apex up.
            b2Vec2 points[b2_maxPolygonVertices];
            for (int i = 0; i < style.sides; ++i) {
                float angle = kPi / 2.0f + 2.0f * kPi * i / style.sides;
                points[i].Set(kPolygonRadius * std::cos(angle), kPolygonRadius * std::sin(angle));
            }
            polygon.Set(points, style.sides);
            fixtureDef.shape = &polygon;
        }
        body->CreateFixture(&fixtureDef);

        return { body, style.r, style.g, style.b };
    }

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({ .width = kWindowWidth, .height = kWindowHeight, .title = "Box2D" });
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world(b2Vec2(0.0f, -kGravity));

        b2BodyDef groundDef;
        groundDef.position.Set(kGroundX, kGroundY);
        b2Body* ground = world.CreateBody(&groundDef);

        b2PolygonShape groundShape;
        groundShape.SetAsBox(kGroundHalfWidth, kGroundHalfHeight);

        b2FixtureDef groundFixture;
        groundFixture.shape = &groundShape;
        groundFixture.friction = 0.6f;
        ground->CreateFixture(&groundFixture);

        std::vector<Item> items;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        int selectedShape = 0;

        float spawnTimer = 0.0f;

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            double now = glfwGetTime();
            float frameTime = std::min(static_cast<float>(now - previousTime), kMaxFrameTime);
            previousTime = now;

            for (int i = 0; i < kShapeCount; ++i) {
                if (window.isKeyPressed(GLFW_KEY_1 + i)) {
                    selectedShape = i;
                }
            }

            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);

            spawnTimer -= frameTime;
            if (mouseDown && spawnTimer <= 0.0f) {
                items.push_back(createItem(world, kShapes[selectedShape],
                    pixelsToMetersX(static_cast<float>(mouseX)),
                    pixelsToMetersY(static_cast<float>(mouseY))));
                spawnTimer = kSpawnCooldown;
            }

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, kVelocityIterations, kPositionIterations);
                accumulator -= kStepDt;
            }

            for (auto it = items.begin(); it != items.end();) {
                if (it->body->GetPosition().y < kRemoveBelowY) {
                    world.DestroyBody(it->body);
                    it = items.erase(it);
                }
                else {
                    ++it;
                }
            }

            glClear(GL_COLOR_BUFFER_BIT);

            drawBoxBody(ground, kGroundHalfWidth, kGroundHalfHeight, 0.35f, 0.35f, 0.4f);
            for (const Item& item : items) {
                drawItem(item);
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