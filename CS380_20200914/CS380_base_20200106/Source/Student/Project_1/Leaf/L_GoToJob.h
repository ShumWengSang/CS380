#pragma once
#include "BehaviorNode.h"

class L_GoToJob : public BaseNode<L_GoToJob>
{
public:
    L_GoToJob();

protected:
    Vec3 jobPosition;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};