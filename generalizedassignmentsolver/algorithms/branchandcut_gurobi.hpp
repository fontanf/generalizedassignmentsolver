#pragma once

#if GUROBI_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct BranchAndCutGurobiOptionalParameters
{
    Info info = Info();

    const Solution* initial_solution = NULL;
};

struct BranchAndCutGurobiOutput: Output
{
    BranchAndCutGurobiOutput(const Instance& instance, Info& info): Output(instance, info) { }
    BranchAndCutGurobiOutput& algorithm_end(Info& info);
};

BranchAndCutGurobiOutput branchandcut_gurobi(
        const Instance& instance, BranchAndCutGurobiOptionalParameters parameters = {});

}

#endif

