#pragma once
#include "BehaviorNode.h"

class L_WolfInRange : public BaseNode<L_WolfInRange>
{
public:
    L_WolfInRange();

protected:
    float rangeSquared;
    std::vector<Agent*> const * workerAgents;
    std::vector<std::pair<Agent*, Vec3>> AdultsInRange;
    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};