#pragma once
#include "BehaviorNode.h"

class C_RandomSelector_OneFail : public BaseNode<C_RandomSelector_OneFail>
{
public:
    C_RandomSelector_OneFail();
protected:
    size_t randomIndex;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    
    void choose_random_node();
    bool check_for_all_failed() const;
    
};