#include <pch.h>
#include "L_AssignJobFour.h"

L_AssignJobFour::L_AssignJobFour() 
{}

void L_AssignJobFour::on_enter()
{
	BehaviorNode::on_leaf_enter();
	agent->get_blackboard().set_value("HasJob", true);
	agent->get_blackboard().set_value("JobPos", Vec3(0, 0, 95));
}

void L_AssignJobFour::on_update(float dt)
{
	on_success();
    display_leaf_text();
}
