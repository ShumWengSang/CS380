#include <pch.h>
#include "L_AssignJobFive.h"

L_AssignJobFive::L_AssignJobFive() 
{}

void L_AssignJobFive::on_enter()
{
	BehaviorNode::on_leaf_enter();
	agent->get_blackboard().set_value("HasJob", true);
	agent->get_blackboard().set_value("JobPos", Vec3(95, 0, 0));
}

void L_AssignJobFive::on_update(float dt)
{
	on_success();
    display_leaf_text();
}
