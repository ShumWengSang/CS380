#include <pch.h>
#include "L_FindClosestHen.h"

L_FindClosestHen::L_FindClosestHen()
{}

void L_FindClosestHen::on_enter()
{
    // get a list of all current hens
    auto& Hens = (agents->get_all_agents_by_type("Adult"));
    // Get our current position
    const auto& currPos = agent->get_position();

    float furtherstDistanceSoFar = std::numeric_limits<float>::max();
    const Agent* closestHen = nullptr;

    // Run through all the hen agents, find the closest hen
    for (const auto& a : Hens)
    {
        const auto& agentPos = a->get_position();
        const float distance = Vec3::DistanceSquared(currPos, agentPos);

        // If this hen is closer then the bsf hen
        if (distance < furtherstDistanceSoFar)
        {
            furtherstDistanceSoFar = distance;
            closestHen = a;
        }
    }

    BehaviorNode::on_leaf_enter();
    if (Hens.empty())
    {
        on_failure();
    }
    else
    {
        agent->get_blackboard().set_value("ClosestHen", closestHen);
        on_success();
    }


}

void L_FindClosestHen::on_update(float dt)
{
    display_leaf_text();
}
