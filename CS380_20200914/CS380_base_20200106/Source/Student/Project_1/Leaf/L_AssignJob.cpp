#include <pch.h>
#include "L_AssignJob.h"

L_AssignJob::L_AssignJob() 
{}

void L_AssignJob::on_enter()
{
	BehaviorNode::on_leaf_enter();
	agent->get_blackboard().set_value("HasJob", true);
	agent->get_blackboard().set_value("JobPos", Vec3(5, 0, 5));
}

void L_AssignJob::on_update(float dt)
{
	on_success();
    display_leaf_text();
}
