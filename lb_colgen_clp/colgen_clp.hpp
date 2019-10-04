#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "knapsack/opt_minknap/minknap.hpp"
#include "knapsack/opt_bellman/bellman.hpp"

namespace gap
{

typedef int64_t ColIdx;

struct ColGenClpOptionalParameters
{
    Info info = Info();

    std::vector<std::vector<std::vector<ItemIdx>>>* columns = NULL;
    std::vector<int>* fixed_alt = NULL; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
};

struct ColGenClpOutput: Output
{
    ColGenClpOutput(const Instance& ins, Info& info): Output(ins, info) { }
    std::vector<std::vector<std::vector<ItemIdx>>> columns;
    std::vector<double> x;
};

ColGenClpOutput lb_colgen_clp(const Instance& ins, ColGenClpOptionalParameters d);

}

#endif

