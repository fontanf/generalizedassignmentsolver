#pragma once

#if CPLEX_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndCutCplexOptionalParameters
{
    Info info = Info();
};

struct BranchAndCutCplexOutput: Output
{
    BranchAndCutCplexOutput(const Instance& ins, Info& info): Output(ins, info) { }
};

BranchAndCutCplexOutput sopt_branchandcut_cplex(const Instance& ins, BranchAndCutCplexOptionalParameters p = {});

}

#endif

