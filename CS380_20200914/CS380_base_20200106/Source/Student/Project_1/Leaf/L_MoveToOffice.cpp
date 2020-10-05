#include <pch.h>
#include "L_MoveToOffice.h"
#include "Agent/BehaviorAgent.h"

void L_MoveToOffice::on_enter()
{
    // set animation, speed, etc

    targetPoint = Vec3(50, 0, 0);
    //agent->look_at_point(targetPoint);

	BehaviorNode::on_leaf_enter();
}

void L_MoveToOffice::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
