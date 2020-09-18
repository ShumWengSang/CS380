#pragma once
#include "BehaviorNode.h"

class L_FindClosestHen : public BaseNode<L_FindClosestHen>
{
public:
    L_FindClosestHen();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};