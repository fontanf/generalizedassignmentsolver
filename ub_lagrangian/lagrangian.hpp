#pragma once

#include "gap/lib/subinstance.hpp"

namespace gap
{

struct LagOut
{
    LagOut(const Instance& ins)
    {
        mult = std::vector<Profit>(ins.item_number(), 0);
        xj   = std::vector<AgentIdx>(ins.item_number(), 0);
        xji  = std::vector<std::vector<int>>(ins.item_number(), std::vector<int>(ins.agent_number(), 0));
    }
    Profit u = -1;
    std::vector<Profit> mult;
    std::vector<std::vector<int>> xji;
    std::vector<AgentIdx> xj;
    std::vector<Profit> zi;
};

LagOut ub_lagrangian(const SubInstance& sub, Info* info = NULL);

}

