#pragma once
#include "BehaviorNode.h"

class L_WorkingJob : public BaseNode<L_WorkingJob>
{
public:
    L_WorkingJob();

protected:
    float timer;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};