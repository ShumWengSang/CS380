#pragma once
#include "BehaviorNode.h"

class L_RandomMovements : public BaseNode<L_RandomMovements>
{
public:
    L_RandomMovements();
    
protected:
    float timer = 0;
    bool reaquireTarget = false;
    Vec3 targetPoint;
    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit() override;
};