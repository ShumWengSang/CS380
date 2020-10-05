#pragma once
#include "BehaviorNode.h"

class L_AssignJobFour : public BaseNode<L_AssignJobFour>
{
public:
    L_AssignJobFour();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};