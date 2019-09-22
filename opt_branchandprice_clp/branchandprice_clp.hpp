#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndPriceClpData
{
    const Instance& ins;
    Solution& sol;
    Cost& lb;
    std::mt19937_64& gen;
    Info info = Info();
};
Solution sopt_branchandprice_clp(BranchAndPriceClpData d);

}

#endif

