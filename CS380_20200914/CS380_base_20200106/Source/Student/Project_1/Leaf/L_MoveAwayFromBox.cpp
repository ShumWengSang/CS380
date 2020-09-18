#include <pch.h>
#include "L_MoveAwayFromBox.h"

L_MoveAwayFromBox::L_MoveAwayFromBox() : targetPoint()
{}

void L_MoveAwayFromBox::on_enter()
{
    targetPoint = agent->get_blackboard().get_value<Vec3>("TargetMovement");

	BehaviorNode::on_leaf_enter();
}

void L_MoveAwayFromBox::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
