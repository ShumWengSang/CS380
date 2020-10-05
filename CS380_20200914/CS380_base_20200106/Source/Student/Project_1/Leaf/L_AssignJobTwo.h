#pragma once
#include "BehaviorNode.h"

class L_AssignJobTwo : public BaseNode<L_AssignJobTwo>
{
public:
    L_AssignJobTwo();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};