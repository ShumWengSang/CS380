#include <pch.h>
#include "L_AssignJobThree.h"

L_AssignJobThree::L_AssignJobThree() 
{}

void L_AssignJobThree::on_enter()
{
	BehaviorNode::on_leaf_enter();
	agent->get_blackboard().set_value("HasJob", true);
	agent->get_blackboard().set_value("JobPos", Vec3(50, 0, 50));
}

void L_AssignJobThree::on_update(float dt)
{
	on_success();
    display_leaf_text();
}
