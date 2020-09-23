#include <pch.h>
#include "L_WolfInRange.h"

L_WolfInRange::L_WolfInRange() : rangeSquared(50.0f), workerAgents(nullptr)
{}

void L_WolfInRange::on_enter()
{
    // get a list of all current wolfs
    workerAgents = &(agents->get_all_agents_by_type("Worker"));
    AdultsInRange.reserve(workerAgents->size());
    
    BehaviorNode::on_leaf_enter();
}

void L_WolfInRange::on_update(float dt)
{
	// Get our current position
	const auto& currPos = agent->get_position();

    // Clear the last frame's wolf in range
    AdultsInRange.clear();

    // Run through all the wolf agents.
	for (const auto& a : *workerAgents)
	{
		const auto& agentPos = a->get_position();
		const float distance = Vec3::DistanceSquared(currPos, agentPos);

        // If wolf is in range
		if (distance < rangeSquared)
		{
            auto direction = agentPos - currPos;
            direction.Normalize();
            AdultsInRange.push_back(std::make_pair(a, direction));
		}
	}
    // If no wolves in range, failure
    if (AdultsInRange.empty())
    {
        on_failure();
        display_leaf_text();
        return;
    }

    // Now from all the wolves in range we determine where we want to go.
    // Average direction of all the wolves
    Vec3 DirectionSum;
    for (const auto& pair : AdultsInRange)
    {
        auto& a = pair.first;
        auto& direction = pair.second;

        DirectionSum += direction;
        
    }
    DirectionSum /= (float)AdultsInRange.size();
    DirectionSum.Normalize();
    DirectionSum *= -1;
    // Move the agent by one movements space of worth
    Vec3 targetPoint = currPos + DirectionSum * agent->get_movement_speed() * 10.0;
    const auto result = agent->move_toward_point(targetPoint, dt);
    
    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
