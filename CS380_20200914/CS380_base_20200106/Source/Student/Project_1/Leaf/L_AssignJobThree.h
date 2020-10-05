#pragma once
#include "BehaviorNode.h"

class L_AssignJobThree : public BaseNode<L_AssignJobThree>
{
public:
    L_AssignJobThree();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};