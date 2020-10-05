#pragma once
#include "BehaviorNode.h"

class L_AssignJobFive : public BaseNode<L_AssignJobFive>
{
public:
    L_AssignJobFive();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};