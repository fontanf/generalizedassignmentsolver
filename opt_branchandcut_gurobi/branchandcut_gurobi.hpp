#pragma once

#if GUROBI_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndCutGurobiData
{
    const Instance& ins;
    Solution& sol;
    Cost& lb;
    Info info = Info();
};

Solution sopt_branchandcut_gurobi(BranchAndCutGurobiData d);

}

#endif

