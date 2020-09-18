#include <pch.h>
#include "D_IsInsideBox.h"
#include "SimpleMath.h"
static float Vec3Angle(Vec3 from, Vec3 to)
{
    float num = (float)sqrtf((double)from.LengthSquared() * (double)to.LengthSquared());
    
    return (double)num < 1.00000000362749E-15 ? 0.0f : (float)acosf((double)std::clamp(from.Dot(to) / num, -1.f, 1.f)) * 57.29578f;
}

static Vec2 Rotate(Vec2 point, float angle, Vec2 center_of_rotation)
{
    float sinus = sin(angle);
    float cosinus = cos(angle);
    Vec2 temp;

    point = point - center_of_rotation;
    temp.x = point.x * cosinus - point.y * sinus;
    temp.y = point.x * sinus + point.y * cosinus;
    point = temp + center_of_rotation;

    return point;
}

D_IsInsideBox::D_IsInsideBox() : boxWidth(5.0f), boxHeight(10.0f), 
    Hen(nullptr), avoidDistance(7.0f)
{}

void D_IsInsideBox::on_enter()
{
    Hen = agent->get_blackboard().get_value<Agent*>("ClosestHen");
}

void D_IsInsideBox::on_update(float dt)
{
    BehaviorNode* child = children.front();
    
    // Generate the hen box by finding the bottom left of the box.
    Vec3 boxPosition = Hen->get_position();
    boxPosition.x -= boxWidth / 2;
    Rect box = Rect(boxPosition.x, boxPosition.y, boxWidth, boxHeight);

    // Find angle to inverse rotate the chick for collision
    Vec3 agentUp = agent->get_up_vector();
    float radianAngle = Vec3Angle(Vec3::Up, agentUp);
    
    // Counter rotate the chick
    Vec3 agentPos = agent->get_position();
    Vec2 newChickPos = Rotate(Vec2(agentPos.x, agentPos.z), radianAngle, { 0,0 });

    // Check new chick position is in the box
    if (box.Contains(newChickPos))
    {
        // agent->get_blackboard().set_value("HenBox", box);
        // Get the normal of which the chick should move to
        Vec3 movementTick;

        // Normal
        Vec2 boxEdgeDir = Vec2(box.x, box.y + box.height) - Vec2(box.x, box.y);
        boxEdgeDir = Vec2(-boxEdgeDir.y, boxEdgeDir.x);
        boxEdgeDir.Normalize();
        
        movementTick = agentPos + Vec3(boxEdgeDir.x, boxEdgeDir.y, agentPos.z);
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
