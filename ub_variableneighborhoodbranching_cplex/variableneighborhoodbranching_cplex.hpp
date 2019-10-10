#pragma once

#if CPLEX_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct VariableNeighborhoodBranchingOptionalParameters
{
    Info info = Info();
};

struct VariableNeighborhoodBranchingOutput: Output
{
    VariableNeighborhoodBranchingOutput(const Instance& ins, Info& info): Output(ins, info) { }
    VariableNeighborhoodBranchingOutput& algorithm_end(Info& info);
};

VariableNeighborhoodBranchingOutput sol_variableneighborhoodbranching_cplex(
        const Instance& ins, std::mt19937_64& gen,
        VariableNeighborhoodBranchingOptionalParameters p = {});

}

#endif

