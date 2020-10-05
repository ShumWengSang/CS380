#include <pch.h>
#include "D_isNotCrazy.h"

D_isNotCrazy::D_isNotCrazy() : delay(0.0f)
{}

void D_isNotCrazy::on_enter()
{
    

    BehaviorNode::on_enter();
}

void D_isNotCrazy::on_update(float dt)
{
	if (agent->get_blackboard().get_value<bool>("IsCrazy"))
	{
		BehaviorNode* child = children.front();

		child->tick(dt);

		// assume same status as child
		set_status(child->get_status());
		set_result(child->get_result());
		return;
	}
	on_failure();
}
