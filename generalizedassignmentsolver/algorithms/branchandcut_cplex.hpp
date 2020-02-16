#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
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

BranchAndCutCplexOutput branchandcut_cplex(const Instance& ins, BranchAndCutCplexOptionalParameters p = {});

}

#endif

