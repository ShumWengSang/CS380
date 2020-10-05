#include <pch.h>
#include "L_RandomMovements.h"

L_RandomMovements::L_RandomMovements() 
{}

void L_RandomMovements::on_enter()
{
	BehaviorNode::on_leaf_enter();
	timer = 5.0f;
	reaquireTarget = true;
	agent->set_movement_speed(50.0f);
}

void L_RandomMovements::on_update(float dt)
{
	timer -= dt;

	if (timer > 0)
	{
		if (reaquireTarget)
		{
			targetPoint = RNG::world_position();
			reaquireTarget = false;
		}
		else
		{
			// Move erratically quickly
			const auto result = agent->move_toward_point(targetPoint, dt);
			if (result == true)
			{
				reaquireTarget = true;
			}
		}
	}
	else
	{
		on_success();
	}

    display_leaf_text();
}

void L_RandomMovements::on_exit()
{
	agent->set_movement_speed(10.0f);
	agent->get_blackboard().set_value("IsCrazy", false);
}
