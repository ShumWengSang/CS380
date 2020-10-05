#pragma once
#include "BehaviorNode.h"

class L_BeCrazy : public BaseNode<L_BeCrazy>
{
public:
    L_BeCrazy();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};