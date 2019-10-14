#pragma once

#if COINOR_FOUND

#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndPriceClpOptionalParameters
{
    Info info = Info();
};

struct BranchAndPriceClpOutput: Output
{
    BranchAndPriceClpOutput(const Instance& ins, Info& info): Output(ins, info) { }
    BranchAndPriceClpOutput& algorithm_end(Info& info);
};

BranchAndPriceClpOutput sopt_branchandprice_clp(const Instance& ins, BranchAndPriceClpOptionalParameters p = {});

}

#endif

