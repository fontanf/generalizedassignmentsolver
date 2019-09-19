#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "knapsack/opt_minknap/minknap.hpp"
#include "knapsack/opt_bellman/bellman.hpp"

namespace gap
{

struct ColGenClpData
{
    const Instance& ins;
    Cost& lb;
    std::vector<std::vector<std::vector<ItemIdx>>>& columns;
    std::vector<AltIdx>& fixed_alt; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
    Info info = Info();
};
Cost lb_colgen_clp(ColGenClpData d);

}

#endif

