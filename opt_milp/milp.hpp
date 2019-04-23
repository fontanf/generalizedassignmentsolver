#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct MilpData
{
    const Instance& ins;
    Solution& sol;
    bool stop_at_first_improvment = false;
    Info info = Info();
};

Solution sopt_milp(MilpData d);

}

