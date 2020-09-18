#include <pch.h>
#include "L_MoveBehindTarget.h"

L_MoveBehindTarget::L_MoveBehindTarget() : Hen(nullptr), behindTargetDistance(3.0f)
{}

void L_MoveBehindTarget::on_enter()
{
    Hen = agent->get_blackboard().get_value<Agent*>("ClosestHen");


	BehaviorNode::on_leaf_enter();
}

void L_MoveBehindTarget::on_update(float dt)
{
    // find an appropriate spot behind the hen
    Vec3 targetPoint;

    // Get the opposite vector of where the hen is facing
    Vec3 oppoDir = Hen->get_forward_vector() * -1;
    oppoDir *= behindTargetDistance;

    targetPoint = oppoDir + Hen->get_position();

    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
