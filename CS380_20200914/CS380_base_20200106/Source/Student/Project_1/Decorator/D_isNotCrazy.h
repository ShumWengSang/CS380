#pragma once
#include "BehaviorNode.h"

class D_isNotCrazy : public BaseNode<D_isNotCrazy>
{
public:
    D_isNotCrazy();

protected:
    float delay;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};