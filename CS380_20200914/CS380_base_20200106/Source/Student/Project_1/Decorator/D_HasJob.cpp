#include <pch.h>
#include "D_HasJob.h"

void D_HasJob::on_update(float dt)
{
    if (agent->get_blackboard().get_value<bool>("HasJob"))
    {
        BehaviorNode* child = children.front();
        child->tick(dt);
        if (child->succeeded() == true)
        {
            on_success();
        }
        else if (child->failed() == true)
        {
            on_failure();
        }
        return;
    }
    on_failure();

}

void D_HasJob::on_exit()
{
    // we want the node and child to repeat, so go back to ready
    set_status(NodeStatus::READY);
}