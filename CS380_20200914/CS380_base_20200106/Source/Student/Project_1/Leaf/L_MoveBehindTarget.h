#pragma once
#include "BehaviorNode.h"

class L_MoveBehindTarget : public BaseNode<L_MoveBehindTarget>
{
public:
    L_MoveBehindTarget();

protected:
    Agent const* Hen;
    float behindTargetDistance;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};