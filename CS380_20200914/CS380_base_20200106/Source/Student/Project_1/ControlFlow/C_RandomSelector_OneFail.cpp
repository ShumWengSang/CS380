#include <pch.h>
#include "C_RandomSelector_OneFail.h"

C_RandomSelector_OneFail::C_RandomSelector_OneFail() : randomIndex(-1)
{}

void C_RandomSelector_OneFail::on_enter()
{
    BehaviorNode::on_enter();

    choose_random_node();
}

void C_RandomSelector_OneFail::on_update(float dt)
{
    // if any child succeeds, the node succeeds
    // if all children fail, the node fails
    BehaviorNode *currentNode = children[randomIndex];
    currentNode->tick(dt);

    if (currentNode->succeeded() == true)
    {
        on_success();
    }
    else if (currentNode->failed() == true)
    {
        on_failure();
    }
}

void C_RandomSelector_OneFail::choose_random_node()
{
    // choose a random node that hasn't succeeded or failed
    while (true)
    {
        randomIndex = RNG::range(static_cast<size_t>(0), children.size() - 1);

        BehaviorNode *node = children[randomIndex];

        if (node->is_ready() == true)
        {
            break;
        }
    }
}

bool C_RandomSelector_OneFail::check_for_all_failed() const
{
    bool result = true;

    for (const auto & child : children)
    {
        result &= child->failed();
    }

    return result;
}
