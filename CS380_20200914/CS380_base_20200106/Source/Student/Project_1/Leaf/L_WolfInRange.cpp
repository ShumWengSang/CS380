#include <pch.h>
#include "L_WolfInRange.h"

L_WolfInRange::L_WolfInRange() : rangeSquared(25.0f), wolfAgents(nullptr)
{}

void L_WolfInRange::on_enter()
{
    // get a list of all current wolfs
    wolfAgents = &(agents->get_all_agents_by_type("Wolf"));
    WolfsInRange.reserve(wolfAgents->size());
    
    BehaviorNode::on_leaf_enter();
}

void L_WolfInRange::on_update(float dt)
{
	// Get our current position
	const auto& currPos = agent->get_position();

    // Clear the last frame's wolf in range
    WolfsInRange.clear();

    // Run through all the wolf agents.
	for (const auto& a : *wolfAgents)
	{
		const auto& agentPos = a->get_position();
		const float distance = Vec3::DistanceSquared(currPos, agentPos);

        // If wolf is in range
		if (distance < rangeSquared)
		{
            auto direction = agentPos - currPos;
            direction.Normalize();
            WolfsInRange.push_back(std::make_pair(a, direction));
		}
	}
    // If no wolves in range, failure
    if (WolfsInRange.empty())
    {
        on_failure();
        display_leaf_text();
        return;
    }

    // Now from all the wolves in range we determine where we want to go.
    // Average direction of all the wolves
    Vec3 DirectionSum;
    for (const auto& pair : WolfsInRange)
    {
        auto& a = pair.first;
        auto& direction = pair.second;

        DirectionSum += direction;
        
    }
    DirectionSum /= WolfsInRange.size();

    // Move the agent by one movements space of worth
    Vec3 targetPoint = currPos + DirectionSum * agent->get_movement_speed();
    const auto result = agent->move_toward_point(targetPoint, dt);
    
    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
