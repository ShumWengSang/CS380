#include <pch.h>
#include "L_WorkingJob.h"

L_WorkingJob::L_WorkingJob() 
{}

void L_WorkingJob::on_enter()
{
    timer = 0;
	BehaviorNode::on_leaf_enter();
}

void L_WorkingJob::on_update(float dt)
{
    timer += dt;
    const float totalTime = 10.0f;
    const float swapSizeInterval = 1.0f;
    if (timer >= totalTime)
    {
        // Mark finish job
        agent->get_blackboard().set_value("HasJob", false);
        on_success();
    }
    else if (fmod(timer, swapSizeInterval) < .1f)
    {
        agent->set_scaling(0.7f);
    }
    else
    {
        agent->set_scaling(0.9f);
    }

    display_leaf_text();
}
