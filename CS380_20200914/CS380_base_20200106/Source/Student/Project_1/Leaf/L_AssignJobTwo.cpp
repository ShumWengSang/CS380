#include <pch.h>
#include "L_AssignJobTwo.h"

L_AssignJobTwo::L_AssignJobTwo() 
{}

void L_AssignJobTwo::on_enter()
{
	BehaviorNode::on_leaf_enter();
	agent->get_blackboard().set_value("HasJob", true);
	agent->get_blackboard().set_value("JobPos", Vec3(95, 0, 95));
}

void L_AssignJobTwo::on_update(float dt)
{
	on_success();
    display_leaf_text();
}
