#pragma once
#include "BehaviorNode.h"

class D_IsInsideBox : public BaseNode<D_IsInsideBox>
{
public:
    D_IsInsideBox();

protected:
    float boxWidth;
    float boxHeight;
    Agent const* Hen;
    float avoidDistance;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};