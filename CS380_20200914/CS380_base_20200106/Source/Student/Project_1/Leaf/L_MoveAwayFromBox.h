#pragma once
#include "BehaviorNode.h"

class L_MoveAwayFromBox : public BaseNode<L_MoveAwayFromBox>
{
public:
    L_MoveAwayFromBox();

protected:
    Vec3 targetPoint;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};