#pragma once

#if CPLEX_FOUND

#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndCutCplexOptionalParameters
{
    Info info = Info();

    const Solution* initial_solution = NULL;
};

struct BranchAndCutCplexOutput: Output
{
    BranchAndCutCplexOutput(const Instance& ins, Info& info): Output(ins, info) { }
    BranchAndCutCplexOutput& algorithm_end(Info& info);
};

BranchAndCutCplexOutput sopt_branchandcut_cplex(const Instance& ins, BranchAndCutCplexOptionalParameters p = {});

}

#endif

