#include <pch.h>
#include "C_Sequencer_Fail.h"

C_Sequencer_Fail::C_Sequencer_Fail() : currentIndex(0)
{}

void C_Sequencer_Fail::on_enter()
{
    currentIndex = 0;
    BehaviorNode::on_enter();
}

void C_Sequencer_Fail::on_update(float dt)
{
    // if any child fails, the node fails
    // if all children succeed, the node succeeds
    BehaviorNode *currentNode = children[currentIndex];
    currentNode->tick(dt);

    if (currentNode->failed() == true)
    {
        on_failure();
    }
    else if (currentNode->succeeded() == true)
    {
        // move to the next node
        ++currentIndex;

        // if we hit the size, then all nodes fail
        if (currentIndex == children.size())
        {
            on_failure();
        }
    }
}
