#pragma once
#include "BehaviorNode.h"

class L_AssignJob : public BaseNode<L_AssignJob>
{
public:
    L_AssignJob();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};