#pragma once
#include "BehaviorNode.h"

class C_Sequencer_Fail : public BaseNode<C_Sequencer_Fail>
{
public:
    C_Sequencer_Fail();
protected:
    size_t currentIndex;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    
};