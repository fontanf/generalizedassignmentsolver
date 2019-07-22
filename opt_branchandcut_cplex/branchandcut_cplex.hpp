#pragma once

#if CPLEX_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndCutCplexData
{
    const Instance& ins;
    Solution& sol;
    Info info = Info();
};

Solution sopt_branchandcut_cplex(BranchAndCutCplexData d);

}

#endif

