#pragma once
#include "BehaviorNode.h"

class D_HasJob : public BaseNode<D_HasJob>
{
protected:
    virtual void on_update(float dt) override;
    virtual void on_exit() override;
};