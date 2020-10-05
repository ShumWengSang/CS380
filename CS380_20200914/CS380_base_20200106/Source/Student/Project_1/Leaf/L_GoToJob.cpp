#include <pch.h>
#include "L_GoToJob.h"

L_GoToJob::L_GoToJob() 
{}

void L_GoToJob::on_enter()
{
    jobPosition = agent->get_blackboard().get_value<Vec3>("JobPos");

	BehaviorNode::on_leaf_enter();
}

void L_GoToJob::on_update(float dt)
{
    const auto result = agent->move_toward_point(jobPosition, dt);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
