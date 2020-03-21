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
    BranchAndCutCplexOutput(const Instance& instance, Info& info): Output(instance, info) { }
    BranchAndCutCplexOutput& algorithm_end(Info& info);
};

BranchAndCutCplexOutput branchandcut_cplex(const Instance& instance, BranchAndCutCplexOptionalParameters p = {});

}

#endif

