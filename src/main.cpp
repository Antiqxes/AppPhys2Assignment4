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
#include<deque>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {
    constexpr int kWindowWidth = 1200;
    constexpr int kWindowHeight = 900;

    constexpr float kPixelsPerMeter = 32.0f;
    // Underwater net gravity (~20% of Earth's 9.81 m/s^2)
    constexpr float kGravity = 2.0f;
    constexpr float kPi = 3.14159265f;

    // Default water drag for all dynamic bodies
    constexpr float kWaterLinearDamping = 2.5f;
    constexpr float kWaterAngularDamping = 5.0f;

    constexpr float kStepDt = 1.0f / 60.0f;
    constexpr float kMaxFrameTime = 0.05f;
    constexpr int kVelocityIterations = 8;
    constexpr int kPositionIterations = 3;

    constexpr float kGroundX = 18.75f;
    constexpr float kGroundY = 1.0f;
    constexpr float kGroundHalfWidth = 18.75f;
    constexpr float kGroundHalfHeight = 0.5f;
    constexpr float kGroundTop = kGroundY + kGroundHalfHeight;

    constexpr uint16 kCategoryGround = 0x0001;
    constexpr uint16 kCategoryBall = 0x0002;
    constexpr uint16 kCategoryBlock = 0x0004;
    constexpr uint16 kCategorySensor = 0x0008;
    constexpr uint16 kCategoryLine = 0x0010;
    constexpr uint16 kCategorySand = 0x0020;

    constexpr int kCircleSegments = 24;

    constexpr float kBallRadius = 0.2f;
    constexpr float kBallDensity = 3.0f;

    constexpr float kMagnetRadius = 0.35f;
    constexpr float kMagnetAccel = 15.0f;     // Acceleration in m/s^2
    constexpr float kMagnetDrag = 4.0f;       // Water drag deceleration when keys are released
    constexpr float kMagnetMaxSpeed = 8.0f;   // Terminal velocity in m/s
    constexpr float kMagnetPullRadius = 6.0f;    // Maximum pull range in meters
    constexpr float kMagnetPullStrength = 35.0f; // Base pull force in Newtons
    constexpr float kMagnetSnapDist = 0.65f;     // Distance at which ball locks onto magnet
    constexpr float kMagnetPuntImpulse = 14.0f; // Launch strength when punting/repelling

    constexpr float kMagnetConeHalfAngleDeg = 15.0f;
    constexpr float kMagnetConeHalfAngleRad = kMagnetConeHalfAngleDeg * (kPi / 180.0f);
    constexpr int kMagnetRayCount = 7; // 7 rays fanned across 30° (every 5°)

    constexpr float kRodAnchorX = 150.0f / kPixelsPerMeter; // 4.6875f meters from left edge
    constexpr float kRodAnchorY = kWindowHeight / kPixelsPerMeter; // 28.125f meters (top edge)
    constexpr float kLineThickness = 2.0f;
    constexpr int kLineSegments = 36;
    constexpr float kLineSegmentHalfLen = 0.35f;
    constexpr float kLineSegmentHalfThick = 0.04f;
    constexpr float kMaxLineLength = kLineSegments * (kLineSegmentHalfLen * 2.0f);

    constexpr int kSandNodeCount = 75;            // 1 node every 0.5m across 37.5m width
    constexpr float kSandRestY = 3.0f;            // Resting height of the sand surface (meters)
    constexpr float kSandFloorY = 0.5f;           // Static bottom floor center Y
    constexpr float kSandNodeRadius = 0.28f;      // Slightly larger than half-spacing so balls can't slip through
    constexpr float kSandLinearDamping = 9.0f;    // High damping so sand settles like wet silt, not water
    constexpr float kSandSpringHz = 2.5f;         // Neighbor stiffness (lower = softer, deeper dents)
    constexpr float kSandDampingRatio = 0.9f;     // Prevents trampoline bouncing
    constexpr float kSandRestoreStiffness = 12.0f; // Higher = levels out faster, Lower = lingers longer

    struct Magnet {
        b2Body* body = nullptr;
        b2Vec2 pos{ 0.0f, 0.0f };
        b2Vec2 vel{ 0.0f, 0.0f };
        b2Vec2 acc{ 0.0f, 0.0f };
        b2Vec2 aimDir{ 1.0f, 0.0f }; // Normalized direction pointing toward mouse
    };

    struct BodyData {
        bool isMagnetic = false;
    };

    struct FishingLine {
        b2Body* anchor = nullptr;
        std::vector<b2Body*> segments;
        b2DistanceJoint* tensionJoint = nullptr;
    };

    struct SandBed {
        b2Body* container = nullptr; // Static bottom, left, and right walls
        std::vector<b2Body*> nodes;  // Dynamic top surface nodes
    };

    constexpr float kWorldWidth = kWindowWidth / kPixelsPerMeter;

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
        fixtureDef.filter.categoryBits = kCategoryGround;
        body->CreateFixture(&fixtureDef);

        return body;
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

    void drawFishingLine(const FishingLine& line, const b2Body* magnetBody) {
        if (line.segments.empty()) return;

        b2Vec2 prevPos = line.anchor->GetPosition();
        for (const b2Body* seg : line.segments) {
            b2Vec2 currPos = seg->GetPosition();
            drawLine(prevPos, currPos, 0.85f, 0.9f, 0.95f);
            prevPos = currPos;
        }
        drawLine(prevPos, magnetBody->GetPosition(), 0.85f, 0.9f, 0.95f);
    }

    void drawSandBed(const SandBed& bed) {
        if (bed.nodes.size() < 2) return;

        float bottomPxY = metersToPixelsY(kSandFloorY);
        float colWidthPx = (kWorldWidth / static_cast<float>(kSandNodeCount - 1)) * kPixelsPerMeter + 1.0f;

        // 1. Fill sand columns from the TOP edge of each node circle down to the static bottom
        for (const b2Body* node : bed.nodes) {
            b2Vec2 pos = node->GetPosition();
            float surfaceY = pos.y + kSandNodeRadius; // Match physical top of circle
            float topPxY = metersToPixelsY(surfaceY);
            float heightPx = std::max(0.0f, bottomPxY - topPxY);
            float centerPxY = topPxY + heightPx * 0.5f;

            gfx::drawBox(metersToPixelsX(pos.x), centerPxY, colWidthPx, heightPx,
                0.0f, 0.76f, 0.65f, 0.42f);
        }

        // 2. Draw the surface crust line along the actual physical contact height
        for (size_t i = 0; i < bed.nodes.size() - 1; ++i) {
            b2Vec2 p1 = bed.nodes[i]->GetPosition();
            b2Vec2 p2 = bed.nodes[i + 1]->GetPosition();
            p1.y += kSandNodeRadius;
            p2.y += kSandNodeRadius;
            drawLine(p1, p2, 0.88f, 0.78f, 0.52f);
        }
    }



    b2BodyDef makeUnderwaterBodyDef(b2Vec2 position, float gravityScale = 1.0f) {
        b2BodyDef def;
        def.type = b2_dynamicBody;
        def.position = position;
        def.linearDamping = kWaterLinearDamping;
        def.angularDamping = kWaterAngularDamping;
        def.gravityScale = gravityScale;
        return def;
    }

    b2Body* createBall(b2World& world, b2Vec2 position, BodyData* data) {
        b2BodyDef bodyDef = makeUnderwaterBodyDef(position);
        bodyDef.bullet = true;
        bodyDef.userData.pointer = reinterpret_cast<uintptr_t>(data);

        b2Body* body = world.CreateBody(&bodyDef);

        b2CircleShape shape;
        shape.m_radius = kBallRadius;

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.density = kBallDensity;
        fixtureDef.friction = 0.4f;
        fixtureDef.restitution = 0.2f;
        fixtureDef.filter.categoryBits = kCategoryBall;
        body->CreateFixture(&fixtureDef);

        return body;
    }

    Magnet createMagnet(b2World& world, b2Vec2 startPos) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;   // Collides with static ground & walls
        bodyDef.position = startPos;
        bodyDef.gravityScale = 0.0f;     // Neutrally buoyant (won't sink)
        bodyDef.fixedRotation = true;    // Keeps magnet upright when carrying balls
        bodyDef.bullet = true;           // Prevents tunneling at max speed
        b2Body* body = world.CreateBody(&bodyDef);

        b2CircleShape shape;
        shape.m_radius = kMagnetRadius;

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.density = 5.0f;       // Heavy enough not to get pushed around easily by small balls
        fixtureDef.friction = 0.2f;      // Low friction so it slides smoothly along ground/walls
        fixtureDef.filter.categoryBits = kCategoryBall;
        body->CreateFixture(&fixtureDef);

        Magnet magnet;
        magnet.body = body;
        magnet.pos = startPos;
        return magnet;
    }

    void updateMagnetVerlet(Magnet& magnet, const gfx::InputState& input, float dt) {
        // 1. Update aim direction toward mouse cursor
        b2Vec2 mouseWorldPos(pixelsToMetersX(input.mousePixelPos.x),
            pixelsToMetersY(input.mousePixelPos.y));
        b2Vec2 toMouse = mouseWorldPos - magnet.body->GetPosition();
        if (toMouse.LengthSquared() > 0.0001f) {
            toMouse.Normalize();
            magnet.aimDir = toMouse;
        }

        // 2. Velocity Verlet Step 1: Half-step velocity & full-step position
        b2Vec2 halfVel = magnet.vel + 0.5f * dt * magnet.acc;
        magnet.pos += dt * halfVel;

        // 3. Compute new acceleration (WASD thrust minus water drag)
        b2Vec2 thrust = kMagnetAccel * input.move;
        b2Vec2 drag = -kMagnetDrag * halfVel;
        b2Vec2 newAcc = thrust + drag;

        // 4. Velocity Verlet Step 2: Complete velocity update & clamp to max speed
        magnet.vel = halfVel + 0.5f * dt * newAcc;
        magnet.acc = newAcc;

        float speed = magnet.vel.Length();
        if (speed > kMagnetMaxSpeed) {
            magnet.vel *= (kMagnetMaxSpeed / speed);
        }

        // 5. Sync velocity with Box2D body
        magnet.body->SetLinearVelocity(magnet.vel);
    }


    class MagnetConeRayCallback : public b2RayCastCallback {
    public:
        b2Body* hitMagneticBody = nullptr;
        float closestFraction = 1.0f;

        float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
            const b2Vec2& normal, float fraction) override {
            if (fixture->IsSensor()) {
                return -1.0f; // Ignore sensors
            }

            b2Body* body = fixture->GetBody();
            uint16 category = fixture->GetFilterData().categoryBits;

            // Walls block the ray completely at this distance
            if (category == kCategoryGround) {
                hitMagneticBody = nullptr;
                closestFraction = fraction;
                return fraction;
            }

            // Record magnetic bodies and clip the ray to their surface
            auto* data = reinterpret_cast<BodyData*>(body->GetUserData().pointer);
            if (data && data->isMagnetic) {
                hitMagneticBody = body;
                closestFraction = fraction;
                return fraction;
            }

            return -1.0f; // Ignore non-magnetic balls, sand, and fishing line
        }
    };

    void drawMagnetCone(const Magnet& magnet, bool active) {
        b2Vec2 origin = magnet.body->GetPosition();
        float baseAngle = std::atan2(magnet.aimDir.y, magnet.aimDir.x);

        if (!active) {
            // Subtle aim indicator when not pulling
            drawLine(origin, origin + 1.0f * magnet.aimDir, 0.4f, 0.6f, 0.7f);
            return;
        }

        float leftAngle = baseAngle + kMagnetConeHalfAngleRad;
        float rightAngle = baseAngle - kMagnetConeHalfAngleRad;

        b2Vec2 leftEnd(origin.x + kMagnetPullRadius * std::cos(leftAngle),
            origin.y + kMagnetPullRadius * std::sin(leftAngle));
        b2Vec2 rightEnd(origin.x + kMagnetPullRadius * std::cos(rightAngle),
            origin.y + kMagnetPullRadius * std::sin(rightAngle));

        // 1. Draw the two outer cone side edges
        drawLine(origin, leftEnd, 0.25f, 0.65f, 0.95f);
        drawLine(origin, rightEnd, 0.25f, 0.65f, 0.95f);

        // 2. Draw a curved arc connecting the outer bounds of the cone
        constexpr int kArcSegments = 8;
        for (int i = 0; i < kArcSegments; ++i) {
            float t1 = static_cast<float>(i) / static_cast<float>(kArcSegments);
            float t2 = static_cast<float>(i + 1) / static_cast<float>(kArcSegments);
            float a1 = rightAngle + t1 * (leftAngle - rightAngle);
            float a2 = rightAngle + t2 * (leftAngle - rightAngle);

            b2Vec2 p1(origin.x + kMagnetPullRadius * std::cos(a1),
                origin.y + kMagnetPullRadius * std::sin(a1));
            b2Vec2 p2(origin.x + kMagnetPullRadius * std::cos(a2),
                origin.y + kMagnetPullRadius * std::sin(a2));
            drawLine(p1, p2, 0.25f, 0.65f, 0.95f);
        }
    }


    std::vector<b2Body*> findMagneticTargetsInCone(b2World& world, const Magnet& magnet) {
        std::vector<b2Body*> targets;
        b2Vec2 origin = magnet.body->GetPosition();
        float baseAngle = std::atan2(magnet.aimDir.y, magnet.aimDir.x);

        for (int i = 0; i < kMagnetRayCount; ++i) {
            float t = -1.0f + 2.0f * static_cast<float>(i) / static_cast<float>(kMagnetRayCount - 1);
            float rayAngle = baseAngle + t * kMagnetConeHalfAngleRad;

            b2Vec2 endPoint(origin.x + kMagnetPullRadius * std::cos(rayAngle),
                origin.y + kMagnetPullRadius * std::sin(rayAngle));

            MagnetConeRayCallback callback;
            world.RayCast(&callback, origin, endPoint);

            if (callback.hitMagneticBody != nullptr) {
                if (std::find(targets.begin(), targets.end(), callback.hitMagneticBody) == targets.end()) {
                    targets.push_back(callback.hitMagneticBody);
                }
            }
        }

        return targets;
    }


    void applyMagnetField(b2World& world, Magnet& magnet,
        std::vector<b2Joint*>& attachedJoints, const gfx::InputState& input) {
        b2Vec2 magnetPos = magnet.body->GetPosition();

        // 1. PUNT / REPEL (Right Click or E)
        if (input.repelTriggered) {
            if (!attachedJoints.empty()) {
                for (b2Joint* joint : attachedJoints) {
                    b2Body* heldBody = joint->GetBodyB();
                    world.DestroyJoint(joint);

                    // Launch straight along mouse aim direction from the front of the magnet
                    b2Vec2 launchPos = magnetPos + (kMagnetRadius + kBallRadius + 0.05f) * magnet.aimDir;
                    heldBody->SetTransform(launchPos, heldBody->GetAngle());
                    heldBody->SetLinearVelocity(magnet.vel);
                    heldBody->ApplyLinearImpulseToCenter(kMagnetPuntImpulse * magnet.aimDir, true);
                }
                attachedJoints.clear();
                return;
            }
            else {
                // Repel only bodies hit by the ±15° mouse-aimed ray fan
                std::vector<b2Body*> targets = findMagneticTargetsInCone(world, magnet);
                for (b2Body* target : targets) {
                    float dist = (target->GetPosition() - magnetPos).Length();
                    float falloff = std::max(0.0f, 1.0f - (dist / kMagnetPullRadius));
                    target->ApplyLinearImpulseToCenter((kMagnetPuntImpulse * falloff) * magnet.aimDir, true);
                }
                return;
            }
        }

        // 2. DROP GENTLY (Neither pull nor repel active)
        if (!input.pullActive) {
            for (b2Joint* joint : attachedJoints) {
                world.DestroyJoint(joint);
            }
            attachedJoints.clear();
            return;
        }

        // 3. PULL & HOLD (Only bodies hit by the ±15° mouse-aimed ray fan)
        std::vector<b2Body*> targets = findMagneticTargetsInCone(world, magnet);
        for (b2Body* target : targets) {
            bool alreadyAttached = false;
            for (b2Joint* joint : attachedJoints) {
                if (joint->GetBodyB() == target) {
                    alreadyAttached = true;
                    break;
                }
            }
            if (alreadyAttached) continue;

            b2Vec2 targetPos = target->GetPosition();
            b2Vec2 toMagnet = magnetPos - targetPos;
            float dist = toMagnet.Length();

            if (dist <= kMagnetSnapDist) {
                b2WeldJointDef weldDef;
                weldDef.Initialize(magnet.body, target, targetPos);
                weldDef.collideConnected = false;
                attachedJoints.push_back(world.CreateJoint(&weldDef));
                continue;
            }

            if (dist > 0.001f) {
                toMagnet.Normalize();
                float falloff = std::max(0.0f, 1.0f - (dist / kMagnetPullRadius));
                target->ApplyForceToCenter((kMagnetPullStrength * falloff) * toMagnet, true);
            }
        }
    }



    FishingLine createFishingLine(b2World& world, b2Vec2 anchorPos, b2Body* magnetBody) {
        FishingLine line;

        // 1. Static rod/boat anchor at the top of the screen
        b2BodyDef anchorDef;
        anchorDef.position = anchorPos;
        line.anchor = world.CreateBody(&anchorDef);

        // 2. Shared shape & filter for thin underwater line segments
        b2PolygonShape segShape;
        segShape.SetAsBox(kLineSegmentHalfThick, kLineSegmentHalfLen);

        b2FixtureDef segFixture;
        segFixture.shape = &segShape;
        segFixture.density = 0.5f;   // Lightweight so it drifts easily in water
        segFixture.friction = 0.1f;
        segFixture.filter.categoryBits = kCategoryLine;
        // Collide only with ground/walls—ignore magnet, balls, and other line links
        segFixture.filter.maskBits = kCategoryGround;

        b2Body* prevBody = line.anchor;
        b2Vec2 prevAnchorLocal(0.0f, 0.0f);

        // 3. Build the chain of segments connected by RevoluteJoints
        for (int i = 0; i < kLineSegments; ++i) {
            float segCenterY = anchorPos.y - (i * 2.0f + 1.0f) * kLineSegmentHalfLen;

            // Low gravityScale (0.2f) so the line floats/billows gently underwater
            b2BodyDef segDef = makeUnderwaterBodyDef(b2Vec2(anchorPos.x, segCenterY), 0.2f);
            segDef.linearDamping = 4.0f; // Extra water drag for smooth billowing
            b2Body* segment = world.CreateBody(&segDef);
            segment->CreateFixture(&segFixture);
            line.segments.push_back(segment);

            b2RevoluteJointDef revDef;
            revDef.bodyA = prevBody;
            revDef.bodyB = segment;
            revDef.localAnchorA = prevAnchorLocal;
            revDef.localAnchorB.Set(0.0f, kLineSegmentHalfLen); // Top end of current segment
            revDef.collideConnected = false;
            world.CreateJoint(&revDef);

            prevBody = segment;
            prevAnchorLocal.Set(0.0f, -kLineSegmentHalfLen); // Bottom end for next link
        }

        // 4. Attach the bottom of the last segment to the center of the magnet
        b2RevoluteJointDef magnetRevDef;
        magnetRevDef.bodyA = prevBody;
        magnetRevDef.bodyB = magnetBody;
        magnetRevDef.localAnchorA = prevAnchorLocal;
        magnetRevDef.localAnchorB.Set(0.0f, 0.0f);
        magnetRevDef.collideConnected = false;
        world.CreateJoint(&magnetRevDef);

        // 5. Add a slack-allowed DistanceJoint so heavy pulls never stretch the chain apart
        b2DistanceJointDef distDef;
        distDef.Initialize(line.anchor, magnetBody, anchorPos, magnetBody->GetPosition());
        distDef.minLength = 0.0f;            // Allows line to go completely slack
        distDef.maxLength = kMaxLineLength;  // Stops magnet cleanly at full line length
        distDef.stiffness = 0.0f;            // Rigid stop at maxLength (no bungee bounce)
        line.tensionJoint = static_cast<b2DistanceJoint*>(world.CreateJoint(&distDef));

        return line;
    }

    SandBed createSandBed(b2World& world) {
        SandBed bed;

        // 1. Create static U-shaped container (bottom floor + left/right walls)
        b2BodyDef containerDef;
        containerDef.position.Set(0.0f, 0.0f);
        bed.container = world.CreateBody(&containerDef);

        b2PolygonShape boxShape;
        b2FixtureDef wallFix;
        wallFix.shape = &boxShape;
        wallFix.friction = 0.8f;
        wallFix.filter.categoryBits = kCategoryGround;

        // Bottom floor across full width (37.5m)
        boxShape.SetAsBox(kWorldWidth * 0.5f, 0.5f, b2Vec2(kWorldWidth * 0.5f, kSandFloorY), 0.0f);
        bed.container->CreateFixture(&wallFix);

        // Left wall (keeps magnet, balls, and sand inside screen)
        float wallHalfHeight = (kWindowHeight / kPixelsPerMeter) * 0.5f;
        boxShape.SetAsBox(0.5f, wallHalfHeight, b2Vec2(-0.5f, wallHalfHeight), 0.0f);
        bed.container->CreateFixture(&wallFix);

        // Right wall
        boxShape.SetAsBox(0.5f, wallHalfHeight, b2Vec2(kWorldWidth + 0.5f, wallHalfHeight), 0.0f);
        bed.container->CreateFixture(&wallFix);

        // 2. Spawn top surface sand nodes
        b2CircleShape nodeShape;
        nodeShape.m_radius = kSandNodeRadius;

        b2FixtureDef nodeFix;
        nodeFix.shape = &nodeShape;
        nodeFix.density = 4.0f;
        nodeFix.friction = 0.9f;
        nodeFix.restitution = 0.0f; // Sand absorbs impact with zero bounce
        nodeFix.filter.categoryBits = kCategorySand;

        float spacing = kWorldWidth / static_cast<float>(kSandNodeCount - 1);

        for (int i = 0; i < kSandNodeCount; ++i) {
            float x = i * spacing;
            // Set gravityScale = 0.0f so the sand bed stays flat and never droops on its own
            b2BodyDef nodeDef = makeUnderwaterBodyDef(b2Vec2(x, kSandRestY), 0.0f);
            nodeDef.linearDamping = kSandLinearDamping;
            nodeDef.fixedRotation = true;

            b2Body* node = world.CreateBody(&nodeDef);
            node->CreateFixture(&nodeFix);
            bed.nodes.push_back(node);

            // Lock each node to vertical-only sliding with floor/ceiling limits
            b2PrismaticJointDef prismDef;
            prismDef.Initialize(bed.container, node, node->GetPosition(), b2Vec2(0.0f, 1.0f));
            prismDef.enableLimit = true;
            prismDef.lowerTranslation = -(kSandRestY - (kSandFloorY + 0.5f + kSandNodeRadius));
            prismDef.upperTranslation = 1.5f;
            world.CreateJoint(&prismDef);
        }

        // 3. Connect adjacent nodes with damped springs so impacts form smooth craters
        for (int i = 0; i < kSandNodeCount - 1; ++i) {
            b2DistanceJointDef springDef;
            springDef.Initialize(bed.nodes[i], bed.nodes[i + 1],
                bed.nodes[i]->GetPosition(), bed.nodes[i + 1]->GetPosition());
            b2LinearStiffness(springDef.stiffness, springDef.damping,
                kSandSpringHz, kSandDampingRatio,
                bed.nodes[i], bed.nodes[i + 1]);
            springDef.collideConnected = false;
            world.CreateJoint(&springDef);
        }

        // Pin leftmost and rightmost nodes to the static walls so the edges stay anchored
        auto pinEdgeNode = [&](b2Body* edgeNode) {
            b2DistanceJointDef edgeSpring;
            edgeSpring.Initialize(bed.container, edgeNode,
                edgeNode->GetPosition(), edgeNode->GetPosition());
            b2LinearStiffness(edgeSpring.stiffness, edgeSpring.damping,
                kSandSpringHz, kSandDampingRatio,
                bed.container, edgeNode);
            world.CreateJoint(&edgeSpring);
            };
        pinEdgeNode(bed.nodes.front());
        pinEdgeNode(bed.nodes.back());

        return bed;
    }

    void updateSandBed(SandBed& bed) {
        for (b2Body* node : bed.nodes) {
            float displacement = kSandRestY - node->GetPosition().y;
            // Only apply force if the node is displaced from rest height
            if (std::abs(displacement) > 0.001f) {
                float restoreForceY = displacement * kSandRestoreStiffness;
                node->ApplyForceToCenter(b2Vec2(0.0f, restoreForceY), true);
            }
        }
    }

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({ .width = kWindowWidth, .height = kWindowHeight, .title = "Box2D" });
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world(b2Vec2(0.0f, -kGravity));

        // Place the ground at the horizontal center of the window using your helper
        float groundCenterX = pixelsToMetersX(kWindowWidth * 0.5f);
        float groundHalfWidth = pixelsToMetersX(kWindowWidth * 0.5f); // Full window width
        float groundY = pixelsToMetersY(kWindowHeight - 32.0f);       // 32px above the bottom edge

        SandBed sandBed = createSandBed(world);

        std::deque<BodyData> bodyDataList;
        std::vector<b2Body*> balls;
        std::vector<b2Joint*> attachedJoints;

        // Helper lambda to spawn a ball with clean memory ownership
        auto spawnBall = [&](b2Vec2 pos, bool isMagnetic) {
            bodyDataList.push_back(BodyData{ .isMagnetic = isMagnetic });
            balls.push_back(createBall(world, pos, &bodyDataList.back()));
            };

        // Spawn one magnetic ball (red) and one non-magnetic ball (gray) to test
        spawnBall(b2Vec2(kWorldWidth * 0.45f, 15.0f), true);
        spawnBall(b2Vec2(kWorldWidth * 0.55f, 15.0f), false);

        Magnet magnet = createMagnet(world, b2Vec2(kRodAnchorX, 14.0f));
        FishingLine fishingLine = createFishingLine(world, b2Vec2(kRodAnchorX, kRodAnchorY), magnet.body);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;

        while (!window.shouldClose()) {
            gfx::InputState input = gfx::processInput(window);

            double now = glfwGetTime();
            float frameTime = std::min(static_cast<float>(now - previousTime), kMaxFrameTime);
            previousTime = now;

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                updateMagnetVerlet(magnet, input, kStepDt);
                applyMagnetField(world, magnet, attachedJoints, input);
                updateSandBed(sandBed);
                world.Step(kStepDt, kVelocityIterations, kPositionIterations);
                magnet.pos = magnet.body->GetPosition();
                magnet.vel = magnet.body->GetLinearVelocity();
                accumulator -= kStepDt;
            }

            glClear(GL_COLOR_BUFFER_BIT);

            drawSandBed(sandBed);

            for (const b2Body* ball : balls) {
                auto* data = reinterpret_cast<BodyData*>(ball->GetUserData().pointer);
                if (data && data->isMagnetic) {
                    drawCircle(ball->GetPosition(), kBallRadius, 1.0f, 0.35f, 0.35f); // Red = magnetic
                }
                else {
                    drawCircle(ball->GetPosition(), kBallRadius, 0.6f, 0.6f, 0.65f);  // Gray = non-magnetic
                }
            }

            // In your render section:
            drawMagnetCone(magnet, input.pullActive);
            drawCircle(magnet.body->GetPosition(), kMagnetRadius, 0.3f, 0.7f, 1.0f);

            // Draw aim indicator line pointing toward the mouse cursor
            b2Vec2 magPos = magnet.body->GetPosition();

            drawFishingLine(fishingLine, magnet.body);
            drawCircle(magnet.body->GetPosition(), kMagnetRadius, 0.3f, 0.7f, 1.0f);


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