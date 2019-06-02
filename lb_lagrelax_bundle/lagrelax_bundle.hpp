#pragma once

#include "gap/lib/subinstance.hpp"

namespace gap
{

struct LagOut
{
    LagOut(const Instance& ins)
    {
        mult = std::vector<Cost>(ins.item_number(), 0);
        xj   = std::vector<AgentIdx>(ins.item_number(), 0);
        xji  = std::vector<std::vector<int>>(ins.item_number(), std::vector<int>(ins.agent_number(), 0));
    }
    Cost u = -1;
    std::vector<Cost> mult;
    std::vector<std::vector<int>> xji;
    std::vector<AgentIdx> xj;
    std::vector<Cost> zi;
};

LagOut lb_lagrangian(const SubInstance& sub,
        StateIdx it_total = 100, StateIdx x = 2, std::vector<Cost>* mult_init = NULL, Info* info = NULL);

}

