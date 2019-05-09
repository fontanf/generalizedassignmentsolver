#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct MilpCplexData
{
    const Instance& ins;
    Solution& sol;
    Info info = Info();
};

Solution sopt_milpcplex(MilpCplexData d);

}

