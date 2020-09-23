#include <pch.h>
#include "D_IsInsideBox.h"
#include "SimpleMath.h"

static float Vec3Angle(Vec3 from, Vec3 to)
{
    float num = (float)sqrtf((float)from.LengthSquared() * (float)to.LengthSquared());
    float rad = (float)num < 1.00000000362749E-15 ? 0.0f : (float)acosf((float)std::clamp(from.Dot(to) / num, -1.f, 1.f));
    float det = from.Cross(to).Dot(Vec3::Up);
    if (det > 0) // Positive
    {
        return fmod(rad, 2 * PI);
    }
    else if (det < 0) // Negative
    {
        return fmod(-rad, 2 * PI);
    }
    else
    {
        return 0;
    }
}

static Vec4 Rotate(Vec4 vector, float radian)
{
    auto RotationMatrix = DirectX::SimpleMath::Matrix::CreateRotationY(radian);
    return DirectX::SimpleMath::Vector4::Transform(vector, RotationMatrix);
}

static Vec4 RotateV2(Vec4 point, float angle, Vec4 center_of_rotation)
{
    Vec4 temp;

    point = point - center_of_rotation;
    temp = Rotate(point, angle);
    point = temp + center_of_rotation;

    return point;
}

static float Area(Vec3 const& a, Vec3 const& b, Vec3 const& c)
{
    return abs((b.x * a.z - a.x * b.z) + (c.x * b.z - b.x * c.z)
        + (a.x * c.z - c.x * a.z)) / 2;
}

struct Box
{
    Vec3 TL;
    Vec3 TR;
    Vec3 BL;
    Vec3 BR;

    float width, height;
public:

    bool isPointInBox(Vec3 point)
    {
        // Calc sum of area APD, DPC, CPB, PBA
        float sum = Area(TL, point, BR) + Area(BR, point, BL) + Area(BL, point, TR) + Area(point, TR, TL);

        // And calc area of rectangle
        float area = width * height;
        return sum <= area;
    }

    void RotateBox(float radians)
    {
        TL = Vec3(Rotate(Vec4(TL.x, TL.y, TL.z, 0), radians));
        TR = Vec3(Rotate(Vec4(TR.x, TR.y, TR.z, 0), radians));
        BL = Vec3(Rotate(Vec4(BL.x, BL.y, BL.z, 0), radians));
        BR = Vec3(Rotate(Vec4(BR.x, BR.y, BR.z, 0), radians));
    }

    void Translate(Vec3 translation)
    {
        TL += translation;
        TR += translation;
        BL += translation;
        BR += translation;
    }
};


D_IsInsideBox::D_IsInsideBox() : boxWidth(8.0f), boxHeight(20.0f), 
    Hen(nullptr), avoidDistance(7.0f)
{}

void D_IsInsideBox::on_enter()
{
    Hen = agent->get_blackboard().get_value<Agent const*>("ClosestHen");
    BehaviorNode::on_enter();
    
}

void D_IsInsideBox::on_update(float dt)
{
    BehaviorNode* child = children.front();

    // Find angle to inverse rotate the chick for collision
    Vec3 HenForward = Hen->get_forward_vector();
    float radianAngle = Vec3Angle(Vec3::Forward, HenForward);

    // Generate the hen box at origin, centered at origin.
    Vec3 BL = Vec3(-boxWidth / 2, 0, 0);
    Vec3 BR = Vec3( boxWidth / 2, 0, 0);
    Vec3 TL = Vec3(-boxWidth / 2, 0, -boxHeight);
    Vec3 TR = Vec3( boxWidth / 2, 0, -boxHeight);
    Box henBox = { TL, TR, BL, BR, boxWidth, boxHeight };

    // Now rotate the box by the angle
    henBox.RotateBox(radianAngle);
    
    // Now translate out to where the player is.
    henBox.Translate(Hen->get_position());

    // Debug draw the rectangle
    renderer->get_debug_renderer().draw_line(henBox.BL, henBox.BR, Color{ 0,177,177 });
    renderer->get_debug_renderer().draw_line(henBox.BL, henBox.TL, Color{ 0,177,177 });
    renderer->get_debug_renderer().draw_line(henBox.BR, henBox.TR, Color{ 0,177,177 });
    renderer->get_debug_renderer().draw_line(henBox.TR, henBox.TL, Color{ 0,177,177 });

    // Check new chick position is in the box
    const Vec3 agentPosition = agent->get_position();
    if (henBox.isPointInBox(agentPosition))
    {
        Vec3 movementTick;

        // Normal
        Vec2 boxEdgeDir = Vec2(henBox.BL - henBox.TL);
        boxEdgeDir = Vec2(-boxEdgeDir.y, boxEdgeDir.x);
        boxEdgeDir.Normalize();

        movementTick = agentPosition + Vec3(boxEdgeDir.x, agentPosition.y, boxEdgeDir.y) * (boxWidth + 5.0f);
        agent->get_blackboard().set_value("TargetMovement", movementTick);

        child->tick(dt);
        if (child->succeeded() == true)
        {
            on_success();
        }
    }
    else
    {
        on_failure();
    }
}
