#include <pch.h>
#include "L_BeCrazy.h"

L_BeCrazy::L_BeCrazy()
{}

void L_BeCrazy::on_enter()
{
    agent->get_blackboard().set_value("IsCrazy", true);

	BehaviorNode::on_leaf_enter();
    on_success();
}

void L_BeCrazy::on_update(float dt)
{

    display_leaf_text();
}
