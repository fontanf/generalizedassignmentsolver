#pragma once

#include "gap/lib/subinstance.hpp"

namespace gap
{

struct LagOut
{
    Profit bound;
    std::vector<Weight> multipliers;
};

LagOut ub_lagrangian(const SubInstance& sub, Info* info = NULL);

}

